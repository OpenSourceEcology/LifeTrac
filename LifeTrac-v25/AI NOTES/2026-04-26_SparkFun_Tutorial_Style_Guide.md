# SparkFun-Style Tutorial Authoring Guide

**Date:** 2026-04-26
**Scope:** Conventions for writing maker-facing build, install, and bring-up guides in this repo (currently `LifeTrac-v25/BUILD-CONTROLLER/`, will extend to `BUILD-HYDRAULIC/` and `BUILD-STRUCTURE/`).
**Reference exemplars in the wild:**
- SparkFun "Qwiic" hookup guides (e.g., *BNO086 Hookup Guide*, *NEO-M9N GPS Hookup Guide*)
- SparkFun "Working with Wire" and "How to Solder: Through-Hole Soldering" tutorials
- Adafruit Learning System guides (very similar structure; secondary reference)

This note codifies what we already adopted in `BUILD-CONTROLLER/02_tractor_node_assembly.md` so future build docs (and other contributors / AI sessions) match the established voice.

---

## 1. Why SparkFun Style?

SparkFun tutorials are the de facto standard for hobbyist electronics documentation because they:

1. Assume a literate beginner — explain *why*, not just *what*.
2. Front-load safety and prerequisites before any irreversible step.
3. Use short, numbered, imperative steps (one verb per step).
4. Mix prose + photo + schematic + code + checklist on one page.
5. End every section with a way to verify success ("you should now see...").

These properties make them auditable, forkable, and friendly to a tired person in a barn at 9 PM. That is exactly the LifeTrac builder.

---

## 2. Document Skeleton

Every BUILD-* tutorial file should follow this skeleton. Sections marked **(required)** are mandatory; **(optional)** can be skipped if not applicable.

```
# <N>. <Title in Title Case>

> **Heads up!** <Top-of-page safety or prerequisite warning>     (required if any safety risk)

<1–3 sentence intro: what this section accomplishes and how it fits in.>     (required)

## What You're Building                                                       (required)
<ASCII diagram or photo + 2-line caption.>

## Required Materials                                                         (required)
<Bulleted list, with link back to the BOM file.>

## Required Tools                                                             (optional)
<Bulleted list, only if section-specific tools beyond the master tool list.>

## Suggested Reading                                                          (optional)
<Bullets with links to prereq tutorials, datasheets, or other BUILD-* sections.>

## Step 1 — <Imperative verb phrase>                                          (required)
1. <One imperative action.>
2. <Next action.>
...
> **Pro tip:** <Optional inline tip>

## Step 2 — ...
...

## Acceptance Test / Verification                                             (required)
- [ ] <Observable success criterion 1>
- [ ] <Observable success criterion 2>
...

## Troubleshooting                                                            (optional)
| Symptom | Likely Cause | Fix |
| ... | ... | ... |

## Resources & Going Further                                                  (optional)
- <Links to next BUILD-* file, datasheets, related AI NOTES.>
```

---

## 3. Voice and Tone

- **Second person, imperative.** "Mount the board," not "The board should be mounted" or "We mount the board."
- **Active, present tense.** "The LED lights up," not "The LED will light up."
- **Concise.** Aim for ≤ 25 words per sentence. Break up anything longer.
- **Friendly but not cute.** No emojis. No exclamation points except inside `Heads up!` callouts. Light humor is okay sparingly (the "ruin your week" line in `02_tractor_node_assembly.md` is at the upper limit).
- **Acknowledge the reader's situation.** "If you've never crimped a Deutsch DT before, do three practice crimps on scrap first."
- **Never apologize for the doc.** Don't write "this section is incomplete" — write what you know, mark TODOs in `TODO.md`.

---

## 4. Callout Conventions

We use three blockquote callout flavors. Use them sparingly — one or two per section, not one per step. Overuse trains the reader to skip them.

### 4.1 Heads up! (Safety / irreversibility)

```markdown
> **Heads up!** <One short sentence stating the hazard or irreversible action,
> then one short sentence on how to avoid it.>
```

Use when:
- Anything that can cause injury (electrical, mechanical, chemical, fall).
- Anything that can destroy a part > $20 (PA damage, reverse polarity, ESD).
- Anything that is hard to undo (conformal coating, soldering SMT, drilling enclosure).

Place at the **top of the section** for whole-section hazards, or **immediately before the dangerous step** for step-local hazards.

### 4.2 Pro tip: (Optional optimization or shortcut)

```markdown
> **Pro tip:** <One sentence with a non-obvious efficiency gain or a workshop trick.>
```

Use for things that are *not* required but make life easier. Never use Pro tip for safety — that's `Heads up!`.

### 4.3 Note: (Clarification or cross-reference)

```markdown
> **Note:** <Clarifying detail, version-specific behavior, or cross-reference to a design doc.>
```

Use when the reader needs context (e.g., "this pinout differs from the rev 1 board"). Used sparingly; most cross-refs go inline as markdown links.

---

## 5. Numbered Steps

- **One verb per step.** "Drill the holes" — not "Drill the holes and install the glands." Split into 2 steps.
- **Imperative mood.** "Mount," "Connect," "Verify."
- **Bold the first noun phrase the user touches.** `1. **Mark the cable-gland positions.** Use a paper template...` This lets the reader scan.
- **Numbered top level, hyphen sub-bullets.** Don't nest numbered lists — markdown renderers disagree on style.
- **Check-back at the end.** End each multi-step section with a one-line "you should now have X" so the reader can self-gate.

Example (from `02_tractor_node_assembly.md` Step 1):

```markdown
## Step 1 — Prep the Enclosure

1. **Mark the cable-gland positions.** Use a paper template (or just measure). Plan for:
   - 1× M20 — main 12 V power feed
   - 2× M16 — RS-485 to remote sensors / valve harness
   ...
2. **Drill** with a step bit at the marked positions. Deburr.
3. **Install glands** finger-tight; you'll torque them down after cabling.
```

---

## 6. Visual Aids

### 6.1 ASCII diagrams (preferred for repo-native docs)

Use ASCII for top-level "what you're building" diagrams. Wrap in a triple-backtick fence (no language tag, or use `text`). Keep them ≤ 80 columns wide. Use box-drawing characters (`┌ ─ ┐ │ └ ┘ ├ ┤ ┬ ┴ ┼ ► ◄`).

Why ASCII first: renders in any markdown viewer, GitHub diffs cleanly, no binary blobs in git, AI tools can read and modify it.

### 6.2 Photos

When a real photo is needed (panel layout, cable routing, "what a good crimp looks like"), reserve a placeholder:

```markdown
![Drilled enclosure, top view, showing M20 power gland in lower-left and breather vent upper-right](images/02_step1_enclosure_drilled.jpg)
```

- File path: `<BUILD-FOLDER>/images/<file_prefix>_<short_name>.jpg`.
- Filename pattern: `<NN>_step<M>_<thing>.jpg` so they sort with their parent doc.
- Always write a real alt-text caption — describe what the reader should see, not "image of enclosure."
- Photos are JPG (max 1600 px on long edge, ~250 KB). Schematics/diagrams are PNG or SVG.

### 6.3 Tables for mappings

Pin maps, channel maps, BOM rows, and troubleshooting all go in markdown tables. Keep them narrow enough to read in a side-by-side diff (≤ 6 columns ideal, ≤ 8 max).

```markdown
| Channel | Net | Function | Notes |
| ------- | --- | -------- | ----- |
| SSR1    | BOOM_UP    | Boom raise valve coil | 1N4007 flyback |
| SSR2    | BOOM_DOWN  | Boom lower valve coil | 1N4007 flyback |
```

### 6.4 Schematics

Inline-link to the canonical schematic file rather than embedding a re-drawing:

```markdown
See [HYDRAULIC_DIAGRAM.md](../DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md) for the full circuit.
```

---

## 7. Code, Commands, and File References

- **Inline `code`** for: file names in flowing prose (`key.h`), commands (`i2cdetect -y 1`), pin names (`PA0`), values (`120 Ω`).
- **Fenced code blocks** with a language tag for: shell commands, source code, config files, register dumps. Always specify the language for syntax highlighting.

  ````markdown
  ```bash
  python3 -c "import secrets; print(secrets.token_hex(16))"
  ```
  ````

- **Shell prompts.** Use `$ ` for unprivileged user, `# ` for root, `>>> ` for Python REPL. Don't include the prompt in copy-paste-friendly multi-line examples (people will copy it and the shell will choke).
- **Filesystem references** in narration are markdown links: `[firmware/common/key.h](../firmware/common/key.h)`. Per repo `fileLinkification` convention: workspace-relative paths, no backticks around the link, no line numbers unless you mean a specific line range.

---

## 8. Linking and Cross-References

Build guides have three audiences for links:

1. **Other build steps** — "After this, see [03_base_station_assembly.md](03_base_station_assembly.md)."
2. **Design docs** — "Per [MASTER_PLAN.md §8.18](../DESIGN-CONTROLLER/MASTER_PLAN.md#818-d1608s-coil-mapping)."
3. **Vendor pages** — "[Phoenix Contact PSR-MC38 at DigiKey](https://www.digikey.com/en/products/detail/...)."

Rules:
- **DigiKey / Mouser first** for purchasable parts. Manufacturer canonical page (Arduino Store, Kurokesu) acceptable as fallback.
- **No fabricated URLs.** If you don't know the exact URL, link to the search results page (`https://www.digikey.com/en/products/result?keywords=ABX00043`) — that survives part-number revisions.
- **Anchor links to design-doc sections** when citing a decision (`#818-d1608s-coil-mapping`). Forces the reader to the authoritative source.
- **Don't deep-link inside vendor PDFs** — they get re-versioned and break.

---

## 9. Acceptance Tests (the "Now Verify" Section)

Every assembly or install section ends with a **checklist of observable, atomic success criteria**. This is the SparkFun "you should now see..." pattern, made explicit.

```markdown
## Acceptance Test

- [ ] Resistance from chassis to battery negative is < 0.1 Ω.
- [ ] With 12 V applied and Opta unpowered, no SSR LED is lit.
- [ ] PSR safety relay K1/K2 contacts are open until the enable input is asserted.
- [ ] Breather vent is installed and finger-tight.
```

Rules:
- **Each item is checkable in < 2 minutes** with the tools listed.
- **Each item is observable** — a multimeter reading, an LED state, a serial-monitor string. Not "system works."
- **Order matches assembly order**, so a failure points at the most recent step.
- **Use GitHub task-list syntax (`- [ ]`)** so contributors can copy a section into an issue and tick boxes.

---

## 10. Bill of Materials Conventions

The BOM file (`01_bill_of_materials.md`) is its own beast. Conventions:

- **Group by tier or subsystem**, not by vendor. Tier 1 = tractor node, Tier 2 = base station, etc.
- **Columns:** Qty | Part | Mfr P/N | Vendor + link | Unit $ | Subtotal $ | Notes.
- **One row per distinct part number.** Sub-variants get their own row.
- **Subtotal each tier.** Grand total at bottom of the file.
- **Substitutions section** at the end, with explicit DO-NOT-SUBSTITUTE warnings for parts where the design depends on a specific behavior (e.g., D1608S vs D1608E — SSR vs EMR is not interchangeable).
- **Region notes.** Call out 915 MHz US vs 868 MHz EU, 110 V vs 230 V supplies, metric vs imperial fasteners.
- **Spares column or section.** Recommend +1 of any single-point-of-failure part.
- **Date the BOM in the file header**, since prices drift.

---

## 11. File Naming and Folder Layout

```
LifeTrac-v25/
└── BUILD-<SUBSYSTEM>/
    ├── README.md                    ← landing page, build-order table, suggested reading
    ├── 01_bill_of_materials.md      ← single source of truth for parts
    ├── 02_<first_assembly>.md
    ├── 03_<next_assembly>.md
    ├── ...
    ├── 0N_firmware_installation.md  ← (controller only) software install
    ├── 0(N+1)_bringup_and_testing.md← final verification gate
    ├── images/                      ← photos and rendered schematics
    └── test_logs/                   ← CSVs from Phase 4-style measurements
```

Rules:
- **Two-digit zero-padded prefix** (`01_`, `02_`, ...) so files sort in build order in any file browser.
- **snake_case filenames**, lowercase.
- **README.md is the entry point** — it links to every numbered file in order.
- **No spaces in filenames** anywhere under `BUILD-*`.
- **Folder name uses HYPHEN, all caps:** `BUILD-CONTROLLER`, not `Build_Controller` or `build-controller`. Matches the existing `DESIGN-*` pattern.

---

## 12. Heading Hierarchy

- `#` — page title only, once per file. Format: `# <N>. <Title>` for numbered chapters; `# <Title>` for README.
- `##` — top-level section (`## Step 1 — ...`, `## Acceptance Test`).
- `###` — sub-section inside a step or callout group.
- `####` — almost never used. If you need it, the section is too long; split into a new chapter.

Rules:
- **No skipped levels** (`##` never directly under `#` is fine, but don't go `##` → `####`).
- **Use em-dash `—` (U+2014)** between step number and title (`## Step 1 — Prep`). Hyphen `-` is acceptable but inconsistent with existing files. Watch out for autocorrect mangling on Windows.
- **Title Case** for `#` and `##`. Sentence case for `###` and below.

---

## 13. Length Targets

| File type | Target length | Hard cap |
| --------- | ------------- | -------- |
| README.md (build folder landing) | 150–250 lines | 400 |
| Numbered chapter | 200–500 lines | 800 |
| BOM | as long as needed | — |
| Bring-up / testing guide | 300–600 lines | 1000 |

If a chapter exceeds the cap, split. Two files at 400 lines each are easier to navigate, edit, and review than one at 800.

---

## 14. What to Avoid

- ❌ **Walls of prose.** If a paragraph runs > 5 lines in rendered view, break it up with a list, table, or sub-heading.
- ❌ **Marketing voice.** "Easily get started with the powerful Portenta X8" → cut.
- ❌ **Vague references.** "Install the latest version" → "Install Arduino IDE 2.x or later (tested on 2.3.6)."
- ❌ **Unverified commands.** Every shell command in the doc must have been run by the author or pulled from a primary source (vendor docs, repo's own scripts).
- ❌ **Hidden state.** Don't assume the reader has just done the previous chapter unless you say so. Each chapter starts with a "Prerequisites" line if needed.
- ❌ **Emojis.** Per repo convention.
- ❌ **TODO comments inline.** Move TODOs to the folder's `TODO.md` or root `TODO.md`. Inline TODOs rot.
- ❌ **Re-stating design rationale.** That belongs in `DESIGN-*/MASTER_PLAN.md`. Build docs say *what to do*, not *why we chose this architecture*. Link back instead.

---

## 15. Authoring Workflow Checklist

When drafting a new BUILD-* chapter:

- [ ] Read the corresponding `DESIGN-*` doc and `MASTER_PLAN.md` sections you're implementing.
- [ ] Confirm part numbers match `01_bill_of_materials.md`. If not, update the BOM first.
- [ ] Write the skeleton from §2 above, leaving step bodies empty.
- [ ] Fill in steps in build order.
- [ ] Add the ASCII diagram in "What You're Building" *after* the steps are written (so it matches reality).
- [ ] Write the Acceptance Test section.
- [ ] Add `Heads up!` callouts only at genuine hazards.
- [ ] Spell-check (Code's built-in or `cspell`).
- [ ] Render in VS Code preview to catch broken links and tables.
- [ ] Cross-link from the folder's `README.md` and the previous/next chapters.
- [ ] Update the folder's `TODO.md` with anything you deferred.

---

## 16. Examples in This Repo

Canonical examples to copy from:

- **Whole-chapter exemplar:** [BUILD-CONTROLLER/02_tractor_node_assembly.md](../BUILD-CONTROLLER/02_tractor_node_assembly.md)
- **Software install exemplar:** [BUILD-CONTROLLER/05_firmware_installation.md](../BUILD-CONTROLLER/05_firmware_installation.md)
- **Verification-gate exemplar:** [BUILD-CONTROLLER/06_bringup_and_testing.md](../BUILD-CONTROLLER/06_bringup_and_testing.md)
- **BOM exemplar:** [BUILD-CONTROLLER/01_bill_of_materials.md](../BUILD-CONTROLLER/01_bill_of_materials.md)
- **Landing-page exemplar:** [BUILD-CONTROLLER/README.md](../BUILD-CONTROLLER/README.md)

When in doubt, open one of these side-by-side with your draft.

---

## 17. References

- SparkFun Tutorials index — https://learn.sparkfun.com/tutorials
- SparkFun "How to Write a Tutorial" (internal contributor guide, public) — https://learn.sparkfun.com/tutorials/how-to-write-a-good-tutorial
- Adafruit Learning System style — https://learn.adafruit.com
- Diátaxis framework (the underlying theory: Tutorial vs How-To vs Reference vs Explanation) — https://diataxis.fr
- This repo's own conventions: [.github/copilot-instructions.md](../../.github/copilot-instructions.md) (if present), `LifeTrac-v25/AI NOTES/conversation_log.md`.

---

**Maintenance note:** When you find yourself wanting to bend one of these rules, do it — and then either (a) update this document to reflect the new rule, or (b) leave a one-line `Note:` in the bending file explaining why. The point is consistency, not religion.
