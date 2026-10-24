"""SIL-only fix prototypes for RS-12.21 (leg 6), applied to a SCRATCH copy.

The worktree's firmware sources are never touched: this copies the
murata_l072 tree into <out root>/<name>/murata_l072 and edits the copy, so
the shipped bench/production builds stay byte-identical to main by
construction. It also writes <out root>/<name>.patch (the diff). Options and
their SIL results: firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md, section 4
(the recommended image is b+L+d2, section 5).
To evaluate a variant (2026-10-24, from PowerShell, WinLibs mingw32-make):
  cd <out root>/<name>/murata_l072
  mingw32-make -k check                      (regressions; the leg-6
                                              characterisation fails by design)
  build/rx_scan_failed_follower -e           (E1 leg-6 replay, E1w watchdog,
                                              E2 cold start, then E3)
  build/rx_scan_failed_follower -e3          (E3 only: the tractor, its
                                              pre-FAILED decay window, FAILED
                                              with and without stalls, E3w)
Written against main 5d00dac2: every edit asserts its anchor text is unique.

usage: py -3 sil_make_variant.py <murata_l072 dir> <out root> <name>[+<name>...]
  a    FAILED + an accepted header -> LOCKED (pure policy)
  b    no absorbing FAILED for a node without originator authority
  d10  scan dwell 10 s (>= one 50-slot epoch) and the walk continues across
       attempts (no restart at ch 0 on a retry)
  d2   scan dwell 2 s, walk continues
  d10r scan dwell 10 s, walker still restarts at ch 0 every attempt
  L    a LOCK always re-anchors the LOCKED loss timer (implied by every d
       option; a alone gets the FAILED-only form)
  noL  leave the loss timer as on main, whatever else is named: shows what L
       fixes (a+noL: the leg-6 check reads locks=2 demotions=1; d10+noL: E1w
       hears 61 % instead of 99.8 %) and what the dwell alone does (d2+noL)
  base no edit (main as it is)
"""
import difflib
import os
import shutil
import sys

SRC, OUT, NAMES = sys.argv[1], sys.argv[2], sys.argv[3]
_names = NAMES.split('+')
for _n in _names:
    if _n not in ('a', 'b', 'd10', 'd2', 'd10r', 'L', 'noL', 'base'):
        raise SystemExit('unknown option ' + _n)
if 'L' in _names and 'noL' in _names:
    raise SystemExit('L and noL together')
COPY =['include', 'radio', 'host', 'hal', 'bench', 'boot', 'ld', 'Makefile',
        'main.c', 'startup.c', 'config.h']

dst = os.path.join(OUT, NAMES, 'murata_l072')
if os.path.exists(dst):
    shutil.rmtree(dst)
os.makedirs(dst)
for c in COPY:
    s = os.path.join(SRC, c)
    if os.path.isdir(s):
        shutil.copytree(s, os.path.join(dst, c))
    elif os.path.exists(s):
        shutil.copy2(s, os.path.join(dst, c))

orig = {}


def edit(rel, old, new):
    p = os.path.join(dst, rel)
    raw = open(p, 'rb').read()
    crlf = b'\r\n' in raw
    s = raw.decode('utf-8').replace('\r\n', '\n')
    orig.setdefault(rel, s)
    n = s.count(old)
    assert n == 1, (rel, n, old[:70])
    s = s.replace(old, new)
    open(p, 'wb').write((s.replace('\n', '\r\n') if crlf else s).encode('utf-8'))


def opt_a():
    edit('radio/sx1276_rx_scan_policy.c',
         '''    case SX1276_RX_SCAN_STATE_FAILED:
    default: {
''',
         '''    case SX1276_RX_SCAN_STATE_FAILED:
        /* PROTO (a), SIL only (RS-12.21 / leg 6): an accepted header is
         * the external recovery. A valid frame heard in FAILED LOCKs
         * exactly as it would from SCANNING; the dispatcher's LOCK path
         * resets the retry budget, and the slot follower takes the synth
         * (LOCKED on the grid consider_remote just adopted). */
        if (event == SX1276_RX_SCAN_EVENT_FRAME_VALID) {
            sx1276_rx_scan_decision_t d = {
                .action     = SX1276_RX_SCAN_ACTION_LOCK,
                .next_state = SX1276_RX_SCAN_STATE_LOCKED,
            };
            return d;
        }
        /* fallthrough */
    default: {
''')


LOCK_ANCHOR_OLD = '''        case SX1276_RX_SCAN_ACTION_HOLD:
        case SX1276_RX_SCAN_ACTION_LOCK:
        case SX1276_RX_SCAN_ACTION_FAIL:
        default:
            /* anchors unchanged */
            break;
'''

LOCK_ANCHOR_FAILED = '''        case SX1276_RX_SCAN_ACTION_LOCK:
            /* PROTO (a), SIL only: a LOCK out of FAILED starts the 2 s
             * loss timer at the locking frame. FAILED's channel anchor is
             * the last walker step, about 0.5 s (one dwell) before the
             * final FAIL -- FAIL leaves the anchors alone -- so without
             * this any LOCK more than about 1.5 s after the final FAIL
             * demotes on the next tick. SCANNING -> LOCK is left as it
             * is. */
            if (s_scan_state == SX1276_RX_SCAN_STATE_FAILED) {
                s_scan_channel_entry_ms = now_ms;
            }
            break;
        case SX1276_RX_SCAN_ACTION_HOLD:
        case SX1276_RX_SCAN_ACTION_FAIL:
        default:
            /* anchors unchanged */
            break;
'''

LOCK_ANCHOR_ALWAYS = '''        case SX1276_RX_SCAN_ACTION_LOCK:
            /* PROTO, SIL only: the LOCKED 2 s loss timer starts at the
             * locking frame, not at the last walker step (up to one dwell
             * old; out of FAILED, one dwell before the final FAIL plus
             * the time since). At a multi-second dwell a LOCK otherwise
             * demotes on the next tick. */
            s_scan_channel_entry_ms = now_ms;
            break;
        case SX1276_RX_SCAN_ACTION_HOLD:
        case SX1276_RX_SCAN_ACTION_FAIL:
        default:
            /* anchors unchanged */
            break;
'''


def lock_anchor(always):
    # Found by this SIL: a LOCK leaves the scan anchors alone, so the LOCKED
    # loss timer runs from the last walker step or attempt start (a+noL:
    # locks=2 demotions=1; d10+noL: E1w hears 61 % of the frames, not 99.8 %).
    edit('radio/sx1276_rx.c', LOCK_ANCHOR_OLD,
         LOCK_ANCHOR_ALWAYS if always else LOCK_ANCHOR_FAILED)


def opt_b():
    edit('radio/sx1276_rx.c',
         '''                if (rx_follow_own_now() == 0U) {
                    (void)sx1276_modes_to_standby();
                }
                return;
            }
''',
         '''                if (rx_follow_own_now() != 0U) {
                    return;
                }
                /* PROTO (b), SIL only (RS-12.21 / leg 6): a node without
                 * originator authority -- a follower, or an originator
                 * whose authority decayed -- never stays FAILED. RX
                 * scanning emits nothing; the 30 s attempt and
                 * its fault (final bit set, attempt nibble at MAX) are
                 * kept, so the host still hears "not acquired" every
                 * 30 s. The retry count stays at MAX: every later FAIL is
                 * final too. */
                s_scan_state               = SX1276_RX_SCAN_STATE_SCANNING;
                s_scan_channel_entry_ms    = now_ms;
                s_scan_cold_start_entry_ms = now_ms;
                scan_dispatch_action(SX1276_RX_SCAN_ACTION_BEGIN_SCAN, now_ms);
                return;
            }
''')
    edit('radio/sx1276_rx.c',
         '''void sx1276_rx_scan_tick(uint32_t now_ms) {
    scan_drive(SX1276_RX_SCAN_EVENT_TICK, false, now_ms);
}
''',
         '''void sx1276_rx_scan_tick(uint32_t now_ms) {
    /* PROTO (b), SIL only: FAILED is held only while the node follows its
     * own clock (originator authority). Once that decays, scan again --
     * on a stalled or stopped TRACTOR too (authority decays 1 s after its
     * last own TX). So once the daemons stop, both boards re-arm RXCONT
     * every dwell indefinitely and the bench park (0x80) holds on
     * neither: (b) needs the RS-12.21 firmware park command, or a
     * park-aware gate, before it can be bench-flown. */
    if (s_scan_state == SX1276_RX_SCAN_STATE_FAILED &&
        sx1276_fhss_is_initialized() != 0U && !sx1276_tx_busy() &&
        rx_follow_own_now() == 0U) {
        sx1276_rx_scan_counter_record(s_scan_state,
                                      SX1276_RX_SCAN_ACTION_BEGIN_SCAN);
        s_scan_state               = SX1276_RX_SCAN_STATE_SCANNING;
        s_scan_channel_entry_ms    = now_ms;
        s_scan_cold_start_entry_ms = now_ms;
        scan_dispatch_action(SX1276_RX_SCAN_ACTION_BEGIN_SCAN, now_ms);
        return;
    }
    scan_drive(SX1276_RX_SCAN_EVENT_TICK, false, now_ms);
}
''')


def dwell(ms):
    # 50 slots of 200 ms (one epoch) = 10 s; d2's 2 s dwell is 10 slots.
    edit('include/sx1276_rx_scan_policy.h',
         '#define SX1276_RX_SCAN_DWELL_MS         500U\n',
         '/* PROTO (d), SIL only: a %g s dwell (%d slots%s) per channel. */\n'
         '#define SX1276_RX_SCAN_DWELL_MS         %dU\n'
         % (ms / 1000.0, ms // 200, ', one epoch' if ms >= 10000 else '', ms))


def walk_continues():
    edit('radio/sx1276_rx.c',
         '''            s_scan_cold_start_entry_ms = now_ms;
            scan_dispatch_action(SX1276_RX_SCAN_ACTION_BEGIN_SCAN, now_ms);
            return;
        }
        case SX1276_RX_SCAN_ACTION_HOLD:''',
         '''            s_scan_cold_start_entry_ms = now_ms;
            /* PROTO (d), SIL only: with multi-second dwells an attempt
             * covers only a few channels -- continue the walk across
             * attempts instead of restarting at ch 0. */
            scan_dispatch_action(SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL, now_ms);
            return;
        }
        case SX1276_RX_SCAN_ACTION_HOLD:''')


for n in NAMES.split('+'):
    if n == 'a':
        opt_a()
    elif n == 'b':
        opt_b()
    elif n == 'd10':
        dwell(10000)
        walk_continues()
    elif n == 'd2':
        dwell(2000)
        walk_continues()
    elif n == 'd10r':
        dwell(10000)
    elif n in ('L', 'noL', 'base'):
        pass
    else:
        raise SystemExit('unknown option ' + n)

if 'noL' in _names:
    pass                        # main's loss timer: shows what L fixes
elif any(n in ('d10', 'd2', 'd10r', 'L') for n in _names):
    lock_anchor(True)
elif 'a' in _names:
    lock_anchor(False)

patch = []
for rel, before in orig.items():
    after = open(os.path.join(dst, rel), 'rb').read().decode('utf-8').replace('\r\n', '\n')
    patch.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                      'a/' + rel, 'b/' + rel))
open(os.path.join(OUT, NAMES + '.patch'), 'w', newline='\n', encoding='utf-8').write(''.join(patch))
print('variant', NAMES, '->', dst, '(%d patched files)' % len(orig))
