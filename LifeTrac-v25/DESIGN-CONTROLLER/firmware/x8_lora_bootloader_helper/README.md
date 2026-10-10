# x8_lora_bootloader_helper

Tools that run on a Portenta X8 to reflash and talk to the Murata L072
(the LoRa radio module on the Max Carrier, on `/dev/ttymxc3`), plus the
PC-side harness for radio bench legs.

Where to start:

- Flashing the L072: [FLASH_RUNBOOK.md](FLASH_RUNBOOK.md).
- Running the radio bench from scratch:
  [bench_tools/BENCH_QUICKSTART.md](bench_tools/BENCH_QUICKSTART.md).
- What is on each bench board today:
  [bench_tools/BENCH_BOARDS.md](bench_tools/BENCH_BOARDS.md).

## Binaries in this folder: read before you flash

The file names here are misleading. None of these files is the firmware
the bench boards run.

| file | size | md5 | what it really is |
|---|---|---|---|
| `mlm32l07x01.bin` | 4,808 B | `1e3d4328dd74e371e4d583e6fe84f856` | **Misnamed.** A LifeTrac test build ("LIFETRAC L072 hello v0.2 (USART1+LPUART1)"): it prints a banner and a tick counter. It is byte-identical to `../murata_l072/build_hello/hello_world.bin`, built from [`../murata_l072/hello_world.c`](../murata_l072/hello_world.c). It is **not** the Murata/Arduino stock firmware, even though it has the stock file name. |
| `hello.bin` | 677 B | `972340b8486fb8589ff99fed3dbf4d29` | An earlier, smaller hello build with the same banner. It is byte-identical to `../murata_l072_hello/hello.bin`, built from [`../murata_l072_hello/main.c`](../murata_l072_hello/main.c). |
| `board2_l072_flash_dump.bin` | 16,440 B | `16e12477f904245f54a694eb7e31fc4c` | A **readback**, not a build. It holds the first 16,440 bytes of L072 flash from `0x08000000`, read in May 2026 (committed 2026-05-13) with [`dump_l072_flash.py`](dump_l072_flash.py) from the board the May notes call "Board 2" (2E2C under that naming). The length is just the size of the `murata_l072` build of that week. It is an early LifeTrac build. It matches no tracked build byte for byte and is not the factory image. Keep it as diagnostic evidence and never flash it. |

### Why the hello build has the stock name

The early flash scripts default to an image called `mlm32l07x01.bin`. For
example, [`run_flash_l072.sh`](run_flash_l072.sh) uses
`IMAGE=${1:-/tmp/lifetrac_p0c/mlm32l07x01.bin}`. The hello build was saved
under that name in May 2026, most likely so those defaults would pick it up
(the 2026-05-08 hello-world notes push it as
`/tmp/lifetrac_p0c/mlm32l07x01.bin`). The file is not renamed here because
old notes and scripts push it by that path. Pass an explicit image path
instead of relying on the default.

### The real stock image and the real bench firmware

- **Stock (factory) L072 firmware:**
  [`LifeTrac-v25/tools/mlm32l07x01.bin`](../../../tools/mlm32l07x01.bin)
  (83,032 B, md5 `33b8a001fac12396c6b7211a463a9cab`). This is Arduino's
  MKRWAN AT-modem firmware: its strings include the `+VER`/`+DEV` AT
  commands, `ARD-078` and version `1.2.3`. It was extracted from the MKRWAN
  library's `MKRWANFWUpdate_standalone/fw.h`. Use it to put a module back to
  factory state; the LifeTrac bench software does not work with it.
- **LifeTrac bench firmware (both bench boards since 2026-09-15):** build
  it with `mingw32-make bench` in [`../murata_l072/`](../murata_l072/). That
  gives `build/firmware_bench_diag.bin`, md5
  `0c1bb0a9573f813137f941dfa47177d0`; toolchain and expected hashes are in
  [`../murata_l072/README.md`](../murata_l072/README.md). Saved copies of
  every flashed bench build are in
  [`bench-evidence/RS_13_vector_scene_2026-09-26/firmware/`](../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md).
