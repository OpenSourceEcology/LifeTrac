# Build configuration — operator & integrator onboarding

> **Round 35 / BC-08.** Single page that ties together every piece of
> the per-unit build-configuration system. If you have just been
> handed a LifeTrac v25 to bring up, edit, or audit, **start here**.
>
> This is the human-narrative companion to four reference documents
> that already exist:
>
> * [`CAPABILITY_INVENTORY.md`](CAPABILITY_INVENTORY.md) — table of
>   every optional/parameterised capability (the *what*).
> * [`base_station/config/build_config.schema.json`](base_station/config/build_config.schema.json)
>   — the machine-readable schema (the *contract*).
> * [`base_station/config/build.default.toml`](base_station/config/build.default.toml)
>   — the canonical default values (the *fallback*).
> * [`HARDWARE_BOM.md`](HARDWARE_BOM.md) — what each capability
>   physically is on a real tractor.
>
> Everything below is normative for operators; the tests in
> [`base_station/tests/test_build_config_doc_sil.py`](base_station/tests/test_build_config_doc_sil.py)
> pin that this page stays in sync with the schema, the CLI, and the
> reload-class vocabulary.

---

## 1. What it is

Every LifeTrac v25 base station boots from a validated **build
configuration** — a TOML file describing the *capabilities* of the
particular tractor it is paired with: how many hydraulic axes, which
E-stop topology, how many cameras, which IMU, which LoRa region,
whether a handheld remote exists, and so on.

A build-config does **not** carry secrets, runtime settings, or
operator preferences. It carries *facts about the hardware that this
particular fleet vehicle was assembled with*. Changing a leaf is
equivalent to changing what the tractor *is*, not what it is *doing*.

The schema is enforced strictly. An invalid config refuses to load
and the base station refuses to boot — by design, because the
firmware downstream (notably the M4 safety core) reads constants
generated from the same source of truth and will trip its watchdog
if the constants disagree with the actual hardware behavior.

---

## 2. Where the files live

| Layer | File | Purpose |
|---|---|---|
| Schema | [`base_station/config/build_config.schema.json`](base_station/config/build_config.schema.json) | JSON Schema draft-07 with `reload_class` annotations on every leaf. |
| Default | [`base_station/config/build.default.toml`](base_station/config/build.default.toml) | Conservative fallback when no per-unit override is present. |
| Per-unit | `base_station/config/build.<unit_id>.toml` | Optional override. Selected by matching `unit_id` against the directory glob. |
| Loader | [`base_station/build_config.py`](base_station/build_config.py) | The only sanctioned entry point. Returns a frozen `BuildConfig` dataclass. |
| Codegen | [`base_station/build_config_codegen.py`](base_station/build_config_codegen.py) | Emits `firmware/common/lifetrac_build_config.h` for the Arduino build. |
| CLI | [`tools/lifetrac_config.py`](../tools/lifetrac_config.py) | Operator-facing wrapper (validate / bundle / verify / diff / push / codegen / dump-json / inventory / self-test). |

**Override resolution order** (first match wins):

1. `LIFETRAC_BUILD_CONFIG_PATH` env var (CI + tests + dev laptops).
2. `base_station/config/build.<unit_id>.toml` matching the loader's
   resolved unit id.
3. `base_station/config/build.default.toml` (the canonical fallback).

---

## 3. The eight sections

Every config file is a TOML document with the following top-level
sections — all eight are **required** and every leaf inside is
required (no implicit defaults; missing keys are rejected at load
time so a typo in a section header cannot silently inherit a default):

| Section | What it pins | Common reasons to override |
|---|---|---|
| **`hydraulic`** | Axis counts, proportional vs bang-bang, ramp seconds. | Drive-only chassis (no arm); cost-down build with bang-bang valves. |
| **`safety`** | E-stop topology, latency budget, watchdog window, Modbus fail-latch. Topology enum: `psr_monitored_dual` (canonical) / `psr_monitored_single` (cost-down) / `hardwired_only` (legacy, refuses to boot together with proportional flow). | Single-channel PSR for cost-down; tighter watchdog after relay upgrade. |
| **`cameras`** | Total count, per-position presence flags, Coral TPU type. | Unit shipped without a rear cam; CPU-only inference build. |
| **`sensors`** | IMU + GPS presence and model; hydraulic pressure-sensor count. | RTK GPS upgrade; no-IMU minimal build. |
| **`comm`** | LoRa region, cellular backup, handheld present. | EU/AU regions; base-station-only build with no handheld. |
| **`ui`** | Web UI enabled, max control subscribers, PIN required. | Demo build with PIN disabled; spectator-heavy fleet review. |
| **`net`** | MQTT host + port. | Off-vehicle broker on the depot LAN. |
| **`aux`** | Auxiliary attachment port count, coupler type, case-drain present. | Build with one or two ISO-5675 couplers and case-drain return. |

The full table of every leaf — type, default, range / enum,
consumer module — lives in
[`CAPABILITY_INVENTORY.md`](CAPABILITY_INVENTORY.md). Read that
first if you are *adding* a capability; read this page if you are
*using* one.

---

## 4. Reload classes

Every leaf carries one of three `reload_class` annotations. They
tell the operator what is required for a change to take effect.

| Class | Meaning | Operator action |
|---|---|---|
| **`live`** | Read on every control tick. | None. The new value is observed within one cycle. |
| **`restart_required`** | Latched at boot or wired into a driver init. | Restart the base-station service (no firmware reflash). |
| **`firmware_required`** | Compiled into the firmware via codegen. | Reflash the M7 + M4 firmware via `lifetrac-config codegen` + the Arduino IDE / `arduino-cli`. |

A `lifetrac-config diff` between the running config and the new
config classifies every change and prints the highest required
action — so the operator never has to guess. Today the
`firmware_required` set is intentionally tiny (`schema_version`
and `safety.m4_watchdog_ms`); everything else lives behind a
`restart_required` boundary or hotter.

---

## 5. The CLI: `lifetrac-config`

All operator-facing operations route through
[`tools/lifetrac_config.py`](../tools/lifetrac_config.py). Run
`python tools/lifetrac_config.py --help` for the canonical synopsis;
the nine subcommands are:

| Subcommand | What it does | Typical use |
|---|---|---|
| `validate` | Schema-validate a TOML or installer bundle. Exits 0 / 2. | Pre-commit hook; CI gate before bundling. |
| `bundle` | Validate + emit `lifetrac-config-<unit_id>-<sha8>.toml` installer bundle. | Operator builds a bundle on the laptop, walks a USB stick to the tractor. |
| `verify` | Re-parse + re-hash a bundle on disk; check filename matches embedded `unit_id` + sha. | Post-USB-copy sanity check on flaky media. |
| `diff` | Show which leaves differ between two configs and what reload class each change requires. | Decide whether a change needs a service restart or a firmware reflash. |
| `push` | Hand a bundle to the X8-side installer daemon for atomic activation. | Deploy from the bench laptop over WiFi when the tractor is online. |
| `codegen` | Emit (or `--check`) the firmware C header from the active config. | CI drift gate; reflash workflow before a `firmware_required` change. |
| `dump-json` | Print the validated config as canonical JSON to stdout. | Non-Python consumers (notably `hil/dispatch.ps1`); shell pipelines. |
| `inventory` | Aggregate `config_loaded` audit events from one or more `audit.jsonl` files into a CSV / Markdown fleet inventory. | Depot operator answering "which fleet vehicles are running which build right now?" |
| `self-test` | Dry-run `boot_self_test.run_self_test` against a TOML config and a JSON-encoded `HardwareInventory`; exit non-zero if any error finding fires. Output as text (default) or JSON. | Pre-deployment bench check before flashing; post-incident replay against a captured inventory dump. |

Every write path uses the atomic `tempfile.mkstemp` + `os.replace`
pattern — there is no half-written config on disk at any moment, so
killing the tool mid-run is always safe.

---

## 6. The deterministic SHA-256

Every loaded config carries a `config_sha256` derived from the canonical
JSON encoding (`sort_keys=True`, `separators=(",", ":")`,
`ensure_ascii=False`). Two configs with the same logical content
always produce the same hash, regardless of TOML key ordering or
whitespace. This SHA appears in:

* The installer bundle filename (`lifetrac-config-<unit_id>-<sha8>.toml`).
* The audit-log `config_loaded` event at every boot.
* The HIL dispatcher's "Applicability source" header line
  (Round 34 / BC-06).
* The `dump-json` payload as the `config_sha256` field.

If the SHA on the bench laptop matches the SHA in the boot log, the
two are byte-identical. No further investigation needed.

---

## 7. Audit trail

On every successful load the base station writes a `config_loaded`
audit-log line carrying `unit_id`, `schema_version`, `config_sha256`,
and the override path that won the resolution race. This is the
canonical answer to "which config was running when X happened".

The HIL dispatcher prints the same identity (`Applicability source:
<unit_id> (sha256=<short>)`) before listing gates, so a saved
dispatch transcript always names the fleet shape it was scoped to.

---

## 8. Common pitfalls

* **UTF-8 BOMs trip `tomllib`.** Notepad / `Set-Content -Encoding
  UTF8` add a BOM; Python rejects it. Use Notepad++ "UTF-8 without
  BOM" or `[System.IO.File]::WriteAllText($p, $body,
  [System.Text.UTF8Encoding]::new($false))`. (Caught Round 34
  during dispatcher smoke-tests.)
* **Section ordering matters for tests, not for the loader.** The
  loader is order-independent, but the
  `BC_D_DeterministicSha256` test rearranges sections assuming
  `[net]` is the last block; new sections (e.g. `[aux]` in Round 33)
  must precede `[net]` in the canonical default file.
* **A new schema leaf needs four edits.** Schema, default TOML,
  loader dataclass, codegen `_SECTIONS` tuple. The
  `test_build_config_loader_sil.py` inventory-parity gate fails
  loudly if any of the four is forgotten.
* **`firmware_required` changes need a reflash.** A `restart` is
  not enough. The `lifetrac-config diff` output is authoritative.

---

## 9. Where to read next

* [`CAPABILITY_INVENTORY.md`](CAPABILITY_INVENTORY.md) — every leaf,
  its consumers, and its rationale.
* [`HARDWARE_BOM.md`](HARDWARE_BOM.md) — physical part for each
  capability.
* [`hil/dispatch.ps1`](hil/dispatch.ps1) +
  [`hil/gate_applicability.json`](hil/gate_applicability.json) —
  how the bench harness uses the active config to skip inapplicable
  gates (Round 34 / BC-06).
* [`MASTER_PLAN.md`](MASTER_PLAN.md) — how this fits into the wider
  controller program.
* [`MASTER_TEST_PROGRAM.md`](../MASTER_TEST_PROGRAM.md)
  — the SIL + HIL gates that pin every contract above.
