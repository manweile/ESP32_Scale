#!/usr/bin/env python3
"""
align_members.py

Aligns C/C++ struct and enum member declarations in files to follow these rules:
1) Indent members by 2 spaces.
2) Place member names 1 space after the longest member data type in the block.
3) Place the equals sign 1 space after the longest member name in the block (i.e., name padded, then ' =').
4) Member value comes 1 space after the equals sign.
5) Any inline comment (`/**< ... */` or `// ...`) should start at column 61 (1-indexed).

Usage:
  python scripts/align_members.py [--in-place] file1.h file2.h ...
  python scripts/align_members.py --diff file1.h

The script uses heuristics and targets simple member lines like:
  <type> <name> [= <value>]; [/**< comment */ | // comment]

It will skip complex lines it cannot parse.
"""

from __future__ import annotations
import re
import sys
import argparse
from typing import List, Tuple
from pathlib import Path
import difflib

# Desired column (1-indexed) where comments should start
COMMENT_COLUMN = 61
INDENT = "  "  # 2 spaces

# Regex to detect the start of a block (struct or enum class)
BLOCK_START_RE = re.compile(r"^\s*(struct|enum\s+class)\b.*\{\s*$")
BLOCK_END_RE = re.compile(r"^\s*\};\s*$")

# Member line regex: capture type, name, optional initializer, semicolon, optional comment
MEMBER_RE = re.compile(
    r"^(?P<indent>\s*)(?P<type>[\w:\<\>\s]+?)\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    r"(?:\s*(?P<assign>=)\s*(?P<value>[^;\/]*?))?"  # optional = value (stop at ; or / start)
    r"\s*;\s*(?P<comment>(?:/\*\*?<.*?\*/|//.*)?)$",
    re.DOTALL,
)

# Enum member regex: name, optional = value, trailing comma, optional comment
ENUM_MEMBER_RE = re.compile(
    r"^\s*(?P<name>[A-Za-z_][A-Za-z0-9_]*)(?:\s*=\s*(?P<value>[^,\/]*?))?\s*,\s*(?P<comment>(?:/\*\*?<.*?\*/|//.*)?)$",
    re.DOTALL,
)

def align_block(lines: List[str], start: int, end: int) -> List[str]:
    """Align member lines between indices [start, end) (inclusive start, exclusive end)."""
    # Determine if this is an enum block by inspecting the header line
    header_line = lines[start - 1] if start - 1 >= 0 else ""
    is_enum = 'enum' in header_line

    new_lines = list(lines)

    if is_enum:
        # Collect enum members
        members = []  # list of (idx, match)
        for i in range(start, end):
            m = ENUM_MEMBER_RE.match(lines[i])
            if m:
                members.append((i, m))
        if not members:
            return lines

        # compute max name length
        names = [m.group('name') for _, m in members]
        max_name = max((len(n) for n in names), default=0)

        for idx, m in members:
            name = m.group('name')
            value = (m.group('value') or '').strip()
            comment = (m.group('comment') or '').rstrip()

            name_field = name.ljust(max_name)
            decl = f"{INDENT}{name_field}"
            if value:
                decl += f" = {value},"
            else:
                decl += ","

            # Position comment at COMMENT_COLUMN
            if comment:
                desired_index = COMMENT_COLUMN - 1
                current_len = len(decl)
                if current_len < desired_index:
                    decl = decl + (" " * (desired_index - current_len)) + comment
                else:
                    decl = decl + " " + comment

            new_lines[idx] = decl + "\n"
        return new_lines

    # struct/block handling
    members: List[Tuple[int, re.Match]] = []
    for i in range(start, end):
        m = MEMBER_RE.match(lines[i])
        if m:
            members.append((i, m))
    if not members:
        return lines

    # Compute column widths
    types = []
    names = []
    for _, m in members:
        types.append(m.group('type').strip())
        names.append(m.group('name'))
    max_type = max((len(t) for t in types), default=0)
    max_name = max((len(n) for n in names), default=0)

    for idx, m in members:
        t = m.group('type').strip()
        name = m.group('name')
        assign = m.group('assign')
        value = m.group('value') or ''
        comment = (m.group('comment') or '').rstrip()

        # Compose declaration
        type_field = t.ljust(max_type + 1)
        name_field = name.ljust(max_name)
        decl = f"{INDENT}{type_field}{name_field}"
        if assign:
            decl += f" = {value.strip()}" if value.strip() else " ="
        decl += ";"

        # Position comment at COMMENT_COLUMN (1-indexed)
        if comment:
            desired_index = COMMENT_COLUMN - 1
            current_len = len(decl)
            if current_len < desired_index:
                spaces = desired_index - current_len
                decl = decl + (" " * spaces) + comment
            else:
                decl = decl + " " + comment

        new_lines[idx] = decl + "\n"

    return new_lines


def process_file(path: Path, in_place: bool) -> Tuple[bool, str]:
    text = path.read_text(encoding='utf-8')
    lines = text.splitlines(keepends=True)

    i = 0
    modified = False
    while i < len(lines):
        if BLOCK_START_RE.match(lines[i]):
            # find end of block
            j = i + 1
            depth = 1
            while j < len(lines):
                if '{' in lines[j]:
                    depth += lines[j].count('{')
                if '}' in lines[j]:
                    depth -= lines[j].count('}')
                    if depth <= 0:
                        break
                j += 1
            # j should be line with '};' or similar; align between i+1 and j
            block_start = i + 1
            block_end = j  # exclusive
            new_lines = align_block(lines, block_start, block_end)
            if new_lines != lines:
                lines = new_lines
                modified = True
            i = j + 1
        else:
            i += 1

    new_text = ''.join(lines)
    if modified and in_place:
        backup = path.with_suffix(path.suffix + '.bak')
        backup.write_text(text, encoding='utf-8')
        path.write_text(new_text, encoding='utf-8')
    return modified, new_text


def main(argv: List[str]):
    parser = argparse.ArgumentParser(description='Align struct/enum member declarations')
    parser.add_argument('files', nargs='+', help='Files to process')
    parser.add_argument('--in-place', action='store_true', help='Edit files in place (creates .bak)')
    parser.add_argument('--diff', action='store_true', help='Show unified diff instead of writing')
    args = parser.parse_args(argv)

    for f in args.files:
        p = Path(f)
        if not p.exists():
            print(f"Skipping missing file: {f}")
            continue
        modified, new_text = process_file(p, in_place=args.in_place)
        if args.diff and modified:
            orig = p.read_text(encoding='utf-8')
            diff = difflib.unified_diff(orig.splitlines(keepends=True), new_text.splitlines(keepends=True), fromfile=str(p), tofile=str(p)+".aligned")
            sys.stdout.writelines(diff)
        elif modified and not args.in_place:
            print(f"{f}: would be modified")
        elif not modified:
            print(f"{f}: unchanged")

if __name__ == '__main__':
    main(sys.argv[1:])
