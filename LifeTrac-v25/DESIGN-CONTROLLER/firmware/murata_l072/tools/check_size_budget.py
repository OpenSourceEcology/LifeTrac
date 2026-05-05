#!/usr/bin/env python3
"""Enforce APP/RAM size budgets for murata_l072 firmware ELF outputs."""

from __future__ import annotations

import argparse
import ast
import pathlib
import re
import subprocess
import sys
from typing import Dict, Set, Tuple

DEFINE_RE = re.compile(r"^#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.+)$")
NUM_SUFFIX_RE = re.compile(r"\b(0[xX][0-9A-Fa-f]+|\d+)(?:[uU][lL]?|[lL][uU]?|[uU]|[lL])\b")
SECTION_RE = re.compile(r"^\s*([.A-Za-z0-9_]+)\s+(\d+)\s+(\d+)\s*$")


class ExprEvaluator(ast.NodeVisitor):
    def __init__(self, eval_symbol):
        self._eval_symbol = eval_symbol

    def visit_Expression(self, node: ast.Expression) -> int:
        return self.visit(node.body)

    def visit_Constant(self, node: ast.Constant) -> int:
        if isinstance(node.value, int):
            return int(node.value)
        raise ValueError(f"Unsupported constant in expression: {node.value!r}")

    def visit_Name(self, node: ast.Name) -> int:
        return self._eval_symbol(node.id)

    def visit_UnaryOp(self, node: ast.UnaryOp) -> int:
        value = self.visit(node.operand)
        if isinstance(node.op, ast.UAdd):
            return +value
        if isinstance(node.op, ast.USub):
            return -value
        if isinstance(node.op, ast.Invert):
            return ~value
        raise ValueError(f"Unsupported unary op: {ast.dump(node.op)}")

    def visit_BinOp(self, node: ast.BinOp) -> int:
        left = self.visit(node.left)
        right = self.visit(node.right)
        op = node.op

        if isinstance(op, ast.Add):
            return left + right
        if isinstance(op, ast.Sub):
            return left - right
        if isinstance(op, ast.Mult):
            return left * right
        if isinstance(op, (ast.Div, ast.FloorDiv)):
            return left // right
        if isinstance(op, ast.Mod):
            return left % right
        if isinstance(op, ast.LShift):
            return left << right
        if isinstance(op, ast.RShift):
            return left >> right
        if isinstance(op, ast.BitOr):
            return left | right
        if isinstance(op, ast.BitAnd):
            return left & right
        if isinstance(op, ast.BitXor):
            return left ^ right

        raise ValueError(f"Unsupported binary op: {ast.dump(op)}")

    def generic_visit(self, node):
        raise ValueError(f"Unsupported expression node: {ast.dump(node)}")


def load_defines(memory_map_path: pathlib.Path) -> Dict[str, str]:
    defines: Dict[str, str] = {}

    for line in memory_map_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line.startswith("#define"):
            continue
        match = DEFINE_RE.match(line)
        if not match:
            continue
        name, expr = match.groups()
        defines[name] = expr.strip()

    return defines


def eval_macro(name: str, defines: Dict[str, str], cache: Dict[str, int], stack: Set[str]) -> int:
    if name in cache:
        return cache[name]
    if name in stack:
        raise ValueError(f"Cyclic macro reference while resolving {name}")
    if name not in defines:
        raise ValueError(f"Macro {name} not found in memory_map.h")

    expr = NUM_SUFFIX_RE.sub(r"\1", defines[name])
    tree = ast.parse(expr, mode="eval")

    stack.add(name)
    evaluator = ExprEvaluator(lambda symbol: eval_macro(symbol, defines, cache, stack))
    value = int(evaluator.visit(tree))
    stack.remove(name)

    cache[name] = value
    return value


def parse_size_sections(elf_path: pathlib.Path) -> Tuple[Dict[str, Tuple[int, int]], str]:
    proc = subprocess.run(
        ["arm-none-eabi-size", "-A", "-d", str(elf_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    if proc.returncode != 0:
        raise RuntimeError(
            "arm-none-eabi-size failed\n"
            f"stdout:\n{proc.stdout}\n"
            f"stderr:\n{proc.stderr}"
        )

    sections: Dict[str, Tuple[int, int]] = {}
    for line in proc.stdout.splitlines():
        match = SECTION_RE.match(line)
        if not match:
            continue
        section_name = match.group(1)
        section_size = int(match.group(2))
        section_addr = int(match.group(3))
        sections[section_name] = (section_size, section_addr)

    return sections, proc.stdout


def fmt_kib(value: int) -> str:
    return f"{value} B ({value / 1024.0:.2f} KiB)"


def main() -> int:
    parser = argparse.ArgumentParser(description="Check firmware size against memory_map budgets.")
    parser.add_argument("elf", type=pathlib.Path, help="Path to firmware ELF (for example build/firmware.elf)")
    parser.add_argument(
        "--memory-map",
        type=pathlib.Path,
        default=pathlib.Path("include/memory_map.h"),
        help="Path to memory_map.h (default: include/memory_map.h)",
    )
    args = parser.parse_args()

    defines = load_defines(args.memory_map)
    cache: Dict[str, int] = {}

    app_base = eval_macro("MM_APP_BASE", defines, cache, set())
    app_size = eval_macro("MM_APP_SIZE", defines, cache, set())
    app_end = app_base + app_size

    boot_base = eval_macro("MM_BOOT_BASE", defines, cache, set())
    boot_size = eval_macro("MM_BOOT_SIZE", defines, cache, set())
    boot_end = boot_base + boot_size

    ram_base = eval_macro("MM_RAM_BASE", defines, cache, set())
    ram_size = eval_macro("MM_RAM_SIZE", defines, cache, set())
    ram_end = ram_base + ram_size
    stack_size = eval_macro("MM_STACK_SIZE", defines, cache, set())

    ram_budget = ram_size - stack_size

    sections, size_stdout = parse_size_sections(args.elf)

    app_used = 0
    boot_used = 0
    ram_used = 0

    for section_name, (section_size, section_addr) in sections.items():
        if section_size == 0:
            continue

        if boot_base <= section_addr < boot_end:
            boot_used += section_size

        if app_base <= section_addr < app_end:
            app_used += section_size

        # .data lives in RAM at runtime but consumes APP flash as a load image.
        if section_name == ".data":
            app_used += section_size

        if ram_base <= section_addr < ram_end:
            ram_used += section_size

    app_ok = app_used <= app_size
    ram_ok = ram_used <= ram_budget

    print("[size-budget] Region usage summary")
    print(f"  BOOT used : {fmt_kib(boot_used)} / {fmt_kib(boot_size)}")
    print(f"  APP used  : {fmt_kib(app_used)} / {fmt_kib(app_size)}")
    print(f"  RAM used  : {fmt_kib(ram_used)} / {fmt_kib(ram_budget)} (stack reserved: {fmt_kib(stack_size)})")

    if app_ok and ram_ok:
        print("[size-budget] PASS")
        return 0

    print("[size-budget] FAIL")
    if not app_ok:
        print(f"  APP over budget by {app_used - app_size} bytes")
    if not ram_ok:
        print(f"  RAM over budget by {ram_used - ram_budget} bytes")

    print("[size-budget] Raw arm-none-eabi-size output:")
    print(size_stdout)
    return 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[size-budget] ERROR: {exc}", file=sys.stderr)
        raise
