"""Check that every markdown table row has the same column count as its header."""
import re
import sys

for path in sys.argv[1:]:
    lines = open(path, encoding="utf-8").read().splitlines()
    in_code = False
    header = None
    for i, line in enumerate(lines, 1):
        if line.startswith("```"):
            in_code = not in_code
        if in_code or not line.startswith("|"):
            header = None if not line.startswith("|") else header
            continue
        # strip backtick spans so '|' inside code does not count; drop escaped pipes
        txt = re.sub(r"`[^`]*`", "``", line.replace("\\|", ""))
        n = txt.count("|")
        if header is None:
            header = n
        elif n != header:
            print(f"{path}:{i}: {n} pipes vs header {header}")
print("done")
