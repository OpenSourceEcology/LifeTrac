#!/bin/sh
# Compare old (HEAD) vs new production ELF: which functions changed?
W=/c/Users/dorkm/Documents/GitHub/LifeTrac/.claude/worktrees/wf_adf2ad4c-341-4
D=/c/Users/dorkm/AppData/Local/Temp/claude/C--Users-dorkm-Documents-GitHub-LifeTrac/5eaec8c2-12ac-4272-80af-b19d1a563f48/scratchpad/imp/fhss-authority
cd "$W" || exit 1
git show HEAD:LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.elf > "$D/old.elf"
cp LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build/firmware.elf "$D/new.elf"
cd "$D"
arm-none-eabi-nm -S --size-sort old.elf | awk '{print $4, $2}' | sort > old.sym
arm-none-eabi-nm -S --size-sort new.elf | awk '{print $4, $2}' | sort > new.sym
echo "=== symbols whose SIZE changed ==="
diff old.sym new.sym
echo "=== authority functions, new ==="
arm-none-eabi-objdump -d --no-show-raw-insn new.elf | awk '/<sx1276_fhss_authority_note_tx>:/,/^$/' | head -40
arm-none-eabi-objdump -d --no-show-raw-insn new.elf | awk '/<sx1276_fhss_authority_is_originator>:/,/^$/' | head -40
echo "=== literal 1500 (0x5dc) / 1000 (0x3e8) in the two functions, old ==="
arm-none-eabi-objdump -d old.elf | awk '/<sx1276_fhss_authority_note_tx>:/,/^$/' | grep -i "3e8\|5dc\|cmp"
arm-none-eabi-objdump -d old.elf | awk '/<sx1276_fhss_authority_is_originator>:/,/^$/' | grep -i "3e8\|5dc\|cmp"
