#!/usr/bin/env python3
"""W2-01 X8-side camera first-light capture (pure stdlib V4L2 ioctl).

Designed for the LifeTrac v25 bench where the Portenta X8 + Arduino
Portenta Max Carrier runs Foundries.io LmP (read-only Yocto image) and
therefore has no `v4l2-ctl`, `ffmpeg`, `gstreamer`, or `python3-opencv`
available out of the box. This script speaks the V4L2 ioctl protocol
directly via `fcntl.ioctl` + `ctypes` + `mmap` so it requires only the
Python 3 stdlib.

Behaviour:
  1. Open the requested /dev/videoN node.
  2. VIDIOC_QUERYCAP - require V4L2_CAP_VIDEO_CAPTURE + V4L2_CAP_STREAMING.
  3. Walk VIDIOC_ENUM_FMT and print every supported pixel format.
  4. VIDIOC_S_FMT to MJPG @ requested resolution (UVC C2 native).
  5. VIDIOC_REQBUFS(1, MMAP) -> VIDIOC_QUERYBUF -> mmap.
  6. VIDIOC_QBUF -> VIDIOC_STREAMON -> poll() -> VIDIOC_DQBUF.
  7. Slice mmap to `bytesused` and write to the requested output file.
  8. VIDIOC_STREAMOFF + munmap + close.

Result is a marker block on stdout that the host orchestrator parses:
  __W2_01_CAMERA_BEGIN__
  ... key=value lines ...
  __W2_01_CAMERA_END__

Exit code 0 = capture file written and non-empty. Non-zero = failure
(detail printed before the END marker if reachable).

Invocation (from host):
  adb -s 2D0A1209DABC240B exec-out python3 /tmp/w2_01_camera_capture.py \\
      --device /dev/video0 --width 1280 --height 720 --out /tmp/w2_01_snap.jpg
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import errno
import os
import select
import struct
import sys
import time
from typing import Iterable

# ---------------------------------------------------------------------------
# libc bindings (Foundries.io LmP ships a stripped Python without the `fcntl`
# or `mmap` stdlib C extensions, so we call libc directly via ctypes which IS
# present as `_ctypes.so`).
# ---------------------------------------------------------------------------
_libc_path = ctypes.util.find_library('c') or 'libc.so.6'
_libc = ctypes.CDLL(_libc_path, use_errno=True)

# int ioctl(int fd, unsigned long request, void *arg);
_libc.ioctl.argtypes = [ctypes.c_int, ctypes.c_ulong, ctypes.c_void_p]
_libc.ioctl.restype = ctypes.c_int

# void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
_libc.mmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
                       ctypes.c_int, ctypes.c_int, ctypes.c_long]
_libc.mmap.restype = ctypes.c_void_p

# int munmap(void *addr, size_t length);
_libc.munmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
_libc.munmap.restype = ctypes.c_int

PROT_READ = 0x1
PROT_WRITE = 0x2
MAP_SHARED = 0x01
MAP_FAILED = ctypes.c_void_p(-1).value


def libc_ioctl(fd: int, request: int, buf) -> None:
    """Mimic fcntl.ioctl(fd, request, buf, mutate_flag=True)."""
    if isinstance(buf, (bytes, bytearray)):
        arr = (ctypes.c_ubyte * len(buf)).from_buffer(buf if isinstance(buf, bytearray) else bytearray(buf))
        rc = _libc.ioctl(fd, request, ctypes.cast(arr, ctypes.c_void_p))
        if rc < 0:
            raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
        # Copy mutated bytes back
        if isinstance(buf, bytearray):
            for i in range(len(buf)):
                buf[i] = arr[i]
        return
    rc = _libc.ioctl(fd, request, buf)
    if rc < 0:
        raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))


class LibcMmap:
    """Minimal mmap.mmap stand-in covering only what we need: __getitem__
    slicing and close()."""
    def __init__(self, fd: int, length: int, offset: int):
        addr = _libc.mmap(None, length, PROT_READ | PROT_WRITE,
                          MAP_SHARED, fd, offset)
        if addr == MAP_FAILED or addr == 0:
            raise OSError(ctypes.get_errno(), os.strerror(ctypes.get_errno()))
        self._addr = addr
        self._length = length
        # Build a ctypes view on the mapped region
        self._view = (ctypes.c_ubyte * length).from_address(addr)

    def __getitem__(self, key):
        if isinstance(key, slice):
            start, stop, step = key.indices(self._length)
            if step != 1:
                return bytes(self._view[start:stop:step])
            return bytes(bytearray(self._view[start:stop]))
        return self._view[key]

    def close(self) -> None:
        if self._addr:
            _libc.munmap(self._addr, self._length)
            self._addr = 0


def libc_mmap(fd: int, length: int, offset: int) -> LibcMmap:
    return LibcMmap(fd, length, offset)


# ---------------------------------------------------------------------------
# V4L2 ioctl constants (from <linux/videodev2.h>, kernel 5.x ABI - stable).
# ---------------------------------------------------------------------------
# _IOC bit layout: dir(2) | size(14) | type(8) | nr(8)
_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_NONE = 0
_IOC_WRITE = 1
_IOC_READ = 2


def _IOC(direction: int, type_: int, nr: int, size: int) -> int:
    return (
        (direction << _IOC_DIRSHIFT)
        | (type_ << _IOC_TYPESHIFT)
        | (nr << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def _IOR(t, nr, size):
    return _IOC(_IOC_READ, t, nr, size)


def _IOW(t, nr, size):
    return _IOC(_IOC_WRITE, t, nr, size)


def _IOWR(t, nr, size):
    return _IOC(_IOC_READ | _IOC_WRITE, t, nr, size)


# v4l2 type byte
V4L2_TYPE = ord('V')

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_FIELD_NONE = 1
V4L2_FIELD_ANY = 0
V4L2_MEMORY_MMAP = 1
V4L2_COLORSPACE_DEFAULT = 0

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000


def fourcc(s: str) -> int:
    s = s.encode('ascii')
    return s[0] | (s[1] << 8) | (s[2] << 16) | (s[3] << 24)


def fourcc_to_str(v: int) -> str:
    return ''.join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


V4L2_PIX_FMT_MJPEG = fourcc('MJPG')
V4L2_PIX_FMT_YUYV = fourcc('YUYV')
V4L2_PIX_FMT_NV12 = fourcc('NV12')


# struct v4l2_capability { __u8 driver[16]; __u8 card[32]; __u8 bus_info[32];
#     __u32 version; __u32 capabilities; __u32 device_caps; __u32 reserved[3]; }
# size = 16+32+32+4+4+4+12 = 104
V4L2_CAPABILITY_FMT = '16s32s32sIII12s'
V4L2_CAPABILITY_SIZE = struct.calcsize(V4L2_CAPABILITY_FMT)
assert V4L2_CAPABILITY_SIZE == 104, V4L2_CAPABILITY_SIZE
VIDIOC_QUERYCAP = _IOR(V4L2_TYPE, 0, V4L2_CAPABILITY_SIZE)


# struct v4l2_fmtdesc { __u32 index; __u32 type; __u32 flags; __u8 description[32];
#     __u32 pixelformat; __u32 mbus_code; __u32 reserved[3]; }
# size = 4+4+4+32+4+4+12 = 64
V4L2_FMTDESC_FMT = 'III32sII12s'
V4L2_FMTDESC_SIZE = struct.calcsize(V4L2_FMTDESC_FMT)
assert V4L2_FMTDESC_SIZE == 64, V4L2_FMTDESC_SIZE
VIDIOC_ENUM_FMT = _IOWR(V4L2_TYPE, 2, V4L2_FMTDESC_SIZE)


# struct v4l2_format is a tagged union; the kernel ABI fixes the overall size
# at sizeof(__u32) + 200 bytes of union body = 204 bytes total.
# For VIDEO_CAPTURE the relevant member is v4l2_pix_format:
#   __u32 width; __u32 height; __u32 pixelformat; __u32 field;
#   __u32 bytesperline; __u32 sizeimage; __u32 colorspace; __u32 priv;
#   __u32 flags; __u32 ycbcr_enc/hsv_enc; __u32 quantization; __u32 xfer_func;
# = 12 * 4 = 48 bytes; pad union to 200.
_V4L2_FORMAT_UNION_SIZE = 200
V4L2_FORMAT_FMT = f'I{_V4L2_FORMAT_UNION_SIZE}s'
V4L2_FORMAT_SIZE = struct.calcsize(V4L2_FORMAT_FMT)
assert V4L2_FORMAT_SIZE == 204, V4L2_FORMAT_SIZE
VIDIOC_G_FMT = _IOWR(V4L2_TYPE, 4, V4L2_FORMAT_SIZE)
VIDIOC_S_FMT = _IOWR(V4L2_TYPE, 5, V4L2_FORMAT_SIZE)


# struct v4l2_requestbuffers { __u32 count; __u32 type; __u32 memory;
#     __u32 capabilities; __u8 flags; __u8 reserved[3]; }
# size = 4+4+4+4+1+3 = 20
V4L2_REQUESTBUFFERS_FMT = 'IIIIB3x'
V4L2_REQUESTBUFFERS_SIZE = struct.calcsize(V4L2_REQUESTBUFFERS_FMT)
assert V4L2_REQUESTBUFFERS_SIZE == 20, V4L2_REQUESTBUFFERS_SIZE
VIDIOC_REQBUFS = _IOWR(V4L2_TYPE, 8, V4L2_REQUESTBUFFERS_SIZE)


# struct v4l2_buffer (single-plane) - simplified:
#   __u32 index; __u32 type; __u32 bytesused; __u32 flags; __u32 field;
#   struct timeval timestamp { time_t tv_sec; long tv_usec; };  // 16 bytes on 64-bit
#   struct v4l2_timecode timecode { __u32 type; __u32 flags; __u8 frames;
#       __u8 seconds; __u8 minutes; __u8 hours; __u8 userbits[4]; };  // 16 bytes
#   __u32 sequence; __u32 memory;
#   union { __u32 offset; unsigned long userptr; struct v4l2_plane *planes;
#           __s32 fd; } m;   // pointer-sized = 8 bytes on aarch64
#   __u32 length; __u32 reserved2; __u32 request_fd_or_reserved;
# Total on aarch64: 4+4+4+4+4 + 16 + 16 + 4+4 + 8 + 4+4+4 = 80
# We pack with explicit 'q' for timeval to be aarch64-correct (time_t = 8B,
# suseconds_t = 8B padded). Using native 'l' under struct would vary by
# platform; we want fixed 64-bit layout matching the kernel ABI on 64-bit.
V4L2_BUFFER_FMT = 'IIIII' + 'qq' + 'IIBBBB4s' + 'II' + 'Q' + 'III' + '4x'
V4L2_BUFFER_SIZE = struct.calcsize(V4L2_BUFFER_FMT)
# Kernel struct on 64-bit Linux has trailing alignment padding -> 88 bytes.
# We expect the format above to also evaluate to 88 with the 4x pad regardless
# of the platform's struct alignment rules.
assert V4L2_BUFFER_SIZE == 88, f"v4l2_buffer size {V4L2_BUFFER_SIZE}"
VIDIOC_QUERYBUF = _IOWR(V4L2_TYPE, 9, V4L2_BUFFER_SIZE)
VIDIOC_QBUF = _IOWR(V4L2_TYPE, 15, V4L2_BUFFER_SIZE)
VIDIOC_DQBUF = _IOWR(V4L2_TYPE, 17, V4L2_BUFFER_SIZE)


# VIDIOC_STREAMON / STREAMOFF take an int (the buf type)
VIDIOC_STREAMON = _IOW(V4L2_TYPE, 18, struct.calcsize('i'))
VIDIOC_STREAMOFF = _IOW(V4L2_TYPE, 19, struct.calcsize('i'))


# struct v4l2_frmsizeenum {
#     __u32 index; __u32 pixel_format; __u32 type;
#     union { struct { __u32 w; __u32 h; } discrete;     // 8B
#             struct { __u32 minw,maxw,stepw,minh,maxh,steph; } stepwise; // 24B
#     };
#     __u32 reserved[2];
# };  // size = 4+4+4 + 24 + 8 = 44
V4L2_FRMSIZE_TYPE_DISCRETE = 1
V4L2_FRMSIZE_TYPE_CONTINUOUS = 2
V4L2_FRMSIZE_TYPE_STEPWISE = 3
V4L2_FRMSIZEENUM_FMT = 'III' + 'IIIIII' + 'II'  # 44 bytes (stepwise covers discrete via overlay)
V4L2_FRMSIZEENUM_SIZE = struct.calcsize(V4L2_FRMSIZEENUM_FMT)
assert V4L2_FRMSIZEENUM_SIZE == 44, V4L2_FRMSIZEENUM_SIZE
VIDIOC_ENUM_FRAMESIZES = _IOWR(V4L2_TYPE, 74, V4L2_FRMSIZEENUM_SIZE)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def emit(key: str, value) -> None:
    """Print one structured key=value line for the orchestrator."""
    print(f"{key}={value}", flush=True)


def query_cap(fd: int) -> dict:
    buf = bytearray(V4L2_CAPABILITY_SIZE)
    libc_ioctl(fd, VIDIOC_QUERYCAP, buf)
    driver, card, bus_info, version, caps, device_caps, _ = struct.unpack(
        V4L2_CAPABILITY_FMT, bytes(buf)
    )
    return {
        'driver': driver.rstrip(b'\x00').decode('ascii', 'replace'),
        'card': card.rstrip(b'\x00').decode('ascii', 'replace'),
        'bus_info': bus_info.rstrip(b'\x00').decode('ascii', 'replace'),
        'version': version,
        'capabilities': caps,
        'device_caps': device_caps if (caps & V4L2_CAP_DEVICE_CAPS) else caps,
    }


def enum_formats(fd: int) -> list[dict]:
    out = []
    for index in range(64):
        buf = bytearray(V4L2_FMTDESC_SIZE)
        struct.pack_into(V4L2_FMTDESC_FMT, buf, 0,
                         index, V4L2_BUF_TYPE_VIDEO_CAPTURE, 0,
                         b'', 0, 0, b'')
        try:
            libc_ioctl(fd, VIDIOC_ENUM_FMT, buf)
        except OSError as e:
            if e.errno == errno.EINVAL:
                break
            raise
        _, _, flags, desc, pixfmt, _mbus, _ = struct.unpack(V4L2_FMTDESC_FMT, bytes(buf))
        out.append({
            'index': index,
            'flags': flags,
            'description': desc.rstrip(b'\x00').decode('ascii', 'replace'),
            'fourcc': fourcc_to_str(pixfmt),
            'pixfmt': pixfmt,
        })
    return out


def enum_framesizes(fd: int, pixfmt: int) -> list[str]:
    """Return list of human-readable size descriptors for the given pixel format.
    Safe: never touches buffers or streaming."""
    out: list[str] = []
    for index in range(64):
        buf = bytearray(V4L2_FRMSIZEENUM_SIZE)
        struct.pack_into('III', buf, 0, index, pixfmt, 0)
        try:
            libc_ioctl(fd, VIDIOC_ENUM_FRAMESIZES, buf)
        except OSError as e:
            if e.errno == errno.EINVAL:
                break
            return out  # swallow other errors silently
        _, _, ftype, w_or_min, maxw_or_h, stepw, minh, maxh, steph, _r0, _r1 = \
            struct.unpack(V4L2_FRMSIZEENUM_FMT, bytes(buf))
        if ftype == V4L2_FRMSIZE_TYPE_DISCRETE:
            # discrete fills first 8 bytes of the union: width, height
            out.append(f'{w_or_min}x{maxw_or_h}')
            # Discrete enumeration is exhaustive via index increment
            continue
        # Stepwise / Continuous: emit one descriptor and stop
        out.append(f'{w_or_min}-{maxw_or_h}/{stepw}x{minh}-{maxh}/{steph}')
        break
    return out


def set_fmt(fd: int, width: int, height: int, pixfmt: int) -> tuple[int, int, int]:
    """Set capture format. Returns (width, height, sizeimage) actually negotiated."""
    union = bytearray(_V4L2_FORMAT_UNION_SIZE)
    struct.pack_into('IIIIIIII', union, 0,
                     width, height, pixfmt, V4L2_FIELD_ANY,
                     0, 0, V4L2_COLORSPACE_DEFAULT, 0)
    buf = bytearray(V4L2_FORMAT_SIZE)
    struct.pack_into(V4L2_FORMAT_FMT, buf, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE, bytes(union))
    libc_ioctl(fd, VIDIOC_S_FMT, buf)
    _, returned_union = struct.unpack(V4L2_FORMAT_FMT, bytes(buf))
    w, h, pf, _field, _bpl, sizeimage = struct.unpack_from('IIIIII', returned_union, 0)
    return (w, h, pf, sizeimage)


def reqbufs(fd: int, count: int) -> int:
    buf = bytearray(V4L2_REQUESTBUFFERS_SIZE)
    struct.pack_into(V4L2_REQUESTBUFFERS_FMT, buf, 0,
                     count, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                     V4L2_MEMORY_MMAP, 0, 0)
    libc_ioctl(fd, VIDIOC_REQBUFS, buf)
    granted, _, _, _, _ = struct.unpack(V4L2_REQUESTBUFFERS_FMT, bytes(buf))
    return granted


def make_buffer_struct(index: int) -> bytearray:
    buf = bytearray(V4L2_BUFFER_SIZE)
    struct.pack_into(V4L2_BUFFER_FMT, buf, 0,
                     index,                         # I index
                     V4L2_BUF_TYPE_VIDEO_CAPTURE,   # I type
                     0,                             # I bytesused
                     0,                             # I flags
                     0,                             # I field
                     0, 0,                          # qq timestamp
                     0, 0, 0, 0, 0, 0, b'\x00\x00\x00\x00',  # timecode
                     0,                             # I sequence
                     V4L2_MEMORY_MMAP,              # I memory
                     0,                             # Q m.offset/userptr
                     0,                             # I length
                     0,                             # I reserved2
                     0)                             # I request_fd
    return buf


def parse_buffer_struct(buf: bytes) -> dict:
    fields = struct.unpack(V4L2_BUFFER_FMT, buf)
    (index, btype, bytesused, flags, field,
     ts_sec, ts_usec,
     tc_type, tc_flags, tc_frames, tc_seconds, tc_minutes, tc_hours, tc_userbits,
     sequence, memory,
     m_union, length, reserved2, req_fd) = fields
    return {
        'index': index, 'bytesused': bytesused, 'flags': flags,
        'sequence': sequence, 'm': m_union, 'length': length,
    }


def query_buf(fd: int, index: int) -> dict:
    buf = make_buffer_struct(index)
    libc_ioctl(fd, VIDIOC_QUERYBUF, buf)
    return parse_buffer_struct(bytes(buf))


def qbuf(fd: int, index: int) -> None:
    buf = make_buffer_struct(index)
    libc_ioctl(fd, VIDIOC_QBUF, buf)


def dqbuf(fd: int) -> dict:
    buf = make_buffer_struct(0)
    libc_ioctl(fd, VIDIOC_DQBUF, buf)
    return parse_buffer_struct(bytes(buf))


def streamon(fd: int) -> None:
    arr = bytearray(struct.pack('i', V4L2_BUF_TYPE_VIDEO_CAPTURE))
    libc_ioctl(fd, VIDIOC_STREAMON, arr)


def streamoff(fd: int) -> None:
    arr = bytearray(struct.pack('i', V4L2_BUF_TYPE_VIDEO_CAPTURE))
    libc_ioctl(fd, VIDIOC_STREAMOFF, arr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def capture_one(args) -> int:
    print("__W2_01_CAMERA_BEGIN__", flush=True)
    emit('schema', 'w2_01_camera/1')
    emit('utc_start', time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    emit('device', args.device)
    emit('requested_width', args.width)
    emit('requested_height', args.height)
    emit('requested_pixfmt', args.pixfmt)
    emit('out_path', args.out)

    try:
        fd = os.open(args.device, os.O_RDWR | os.O_NONBLOCK)
    except OSError as e:
        emit('error', f'open_failed errno={e.errno} ({errno.errorcode.get(e.errno, "?")})')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        return 2

    # 1. QUERYCAP
    try:
        cap = query_cap(fd)
    except OSError as e:
        emit('error', f'querycap_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 3

    emit('driver', cap['driver'])
    emit('card', cap['card'])
    emit('bus_info', cap['bus_info'])
    emit('caps_hex', f"0x{cap['capabilities']:08x}")
    emit('device_caps_hex', f"0x{cap['device_caps']:08x}")
    has_capture = bool(cap['device_caps'] & V4L2_CAP_VIDEO_CAPTURE)
    has_streaming = bool(cap['device_caps'] & V4L2_CAP_STREAMING)
    emit('has_video_capture', int(has_capture))
    emit('has_streaming', int(has_streaming))

    if not (has_capture and has_streaming):
        emit('error', 'device_lacks_capture_or_streaming')
        emit('verdict', 'SKIP_WRONG_NODE')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 4

    # 2. ENUM_FMT
    fmts = enum_formats(fd)
    emit('format_count', len(fmts))
    for f in fmts:
        emit(f'fmt_{f["index"]}', f'{f["fourcc"]}|{f["description"]}')
    available_fourccs = [f['fourcc'] for f in fmts]

    # 2b. ENUM_FRAMESIZES per format (safe - no streaming, no buffers).
    for f in fmts:
        sizes = enum_framesizes(fd, f['pixfmt'])
        if sizes:
            emit(f'sizes_{f["fourcc"]}', ';'.join(sizes))

    # SAFE EXIT POINT: in enumerate-only mode we stop here, BEFORE touching
    # REQBUFS / STREAMON / mmap. This avoids the uvcvideo+ci_hdrc wedge that
    # has bricked the X8 USB host on prior bench runs.
    if getattr(args, 'enumerate_only', False):
        emit('verdict', 'PASS_ENUMERATE_ONLY')
        emit('utc_end', time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 0

    # 3. Pick pixel format. Prefer requested, else MJPG, else first.
    pix_str = args.pixfmt.upper()
    if pix_str in available_fourccs:
        chosen = pix_str
    elif 'MJPG' in available_fourccs:
        chosen = 'MJPG'
    else:
        chosen = available_fourccs[0] if available_fourccs else 'YUYV'
    chosen_pixfmt = fourcc(chosen)
    emit('chosen_fourcc', chosen)

    # 4. S_FMT
    try:
        nw, nh, npf, sizeimage = set_fmt(fd, args.width, args.height, chosen_pixfmt)
    except OSError as e:
        emit('error', f'set_fmt_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 5
    emit('negotiated_width', nw)
    emit('negotiated_height', nh)
    emit('negotiated_fourcc', fourcc_to_str(npf))
    emit('negotiated_sizeimage', sizeimage)

    # 5. REQBUFS(1)
    try:
        granted = reqbufs(fd, 1)
    except OSError as e:
        emit('error', f'reqbufs_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 6
    emit('reqbufs_granted', granted)
    if granted < 1:
        emit('error', 'no_buffers_granted')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 7

    # 6. QUERYBUF + mmap
    try:
        qb = query_buf(fd, 0)
    except OSError as e:
        emit('error', f'querybuf_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 8
    buf_offset = qb['m']
    buf_length = qb['length']
    emit('buf_offset', buf_offset)
    emit('buf_length', buf_length)

    try:
        mm = libc_mmap(fd, buf_length, buf_offset)
    except OSError as e:
        emit('error', f'mmap_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        os.close(fd)
        return 9

    # 7. QBUF + STREAMON + poll + DQBUF
    try:
        qbuf(fd, 0)
        streamon(fd)
    except OSError as e:
        emit('error', f'qbuf_or_streamon_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        mm.close()
        os.close(fd)
        return 10

    deadline = time.monotonic() + args.timeout
    captured = None
    captured_bytes = 0
    capture_attempts = 0
    last_errno = None
    # Skip the first few frames - UVC AE/AGC needs settle time. We dequeue and
    # requeue up to args.warmup_frames, then keep the next one.
    keep_after = max(0, args.warmup_frames)
    while time.monotonic() < deadline:
        rlist, _, _ = select.select([fd], [], [], 1.0)
        if not rlist:
            continue
        try:
            db = dqbuf(fd)
        except OSError as e:
            last_errno = e.errno
            if e.errno in (errno.EAGAIN, errno.EINTR):
                continue
            break
        capture_attempts += 1
        if capture_attempts <= keep_after:
            try:
                qbuf(fd, db['index'])
            except OSError as e:
                last_errno = e.errno
                break
            continue
        bytesused = db['bytesused']
        captured = bytes(mm[:bytesused])
        captured_bytes = bytesused
        break

    try:
        streamoff(fd)
    except OSError:
        pass
    mm.close()
    os.close(fd)

    emit('capture_attempts', capture_attempts)
    if captured is None:
        emit('error', f'no_frame_captured last_errno={last_errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        return 11

    # 8. Write file
    try:
        with open(args.out, 'wb') as fh:
            fh.write(captured)
    except OSError as e:
        emit('error', f'write_failed errno={e.errno}')
        emit('verdict', 'FAIL')
        print("__W2_01_CAMERA_END__", flush=True)
        return 12

    emit('captured_bytes', captured_bytes)
    head_hex = captured[:16].hex()
    emit('head16_hex', head_hex)

    # JPEG sanity for MJPG
    is_jpeg = captured[:2] == b'\xff\xd8' and captured[-2:] == b'\xff\xd9'
    emit('jpeg_soi_eoi_ok', int(is_jpeg))

    # Verdict: PASS if we wrote >0 bytes; STRONG_PASS if also JPEG-sane (when MJPG).
    if captured_bytes <= 0:
        emit('verdict', 'FAIL')
        rc = 13
    elif fourcc_to_str(npf) == 'MJPG' and not is_jpeg:
        emit('verdict', 'WARN_NON_JPEG_PAYLOAD')
        rc = 0
    else:
        emit('verdict', 'PASS')
        rc = 0

    emit('utc_end', time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()))
    print("__W2_01_CAMERA_END__", flush=True)
    return rc


def main() -> int:
    p = argparse.ArgumentParser(description='W2-01 X8 camera first-light capture.')
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--pixfmt', default='MJPG',
                   help='4-char fourcc preference (MJPG, YUYV, NV12). Falls back if absent.')
    p.add_argument('--out', default='/tmp/w2_01_snap.bin')
    p.add_argument('--timeout', type=float, default=5.0,
                   help='Seconds to wait for a frame before giving up.')
    p.add_argument('--warmup-frames', type=int, default=4,
                   help='Frames to dequeue+requeue before keeping one (UVC AE settle).')
    p.add_argument('--enumerate-only', action='store_true',
                   help='Stop after QUERYCAP+ENUM_FMT+ENUM_FRAMESIZES. Does NOT call '
                        'REQBUFS/STREAMON/mmap, so cannot wedge the USB host.')
    args = p.parse_args()
    return capture_one(args)


if __name__ == '__main__':
    sys.exit(main())
