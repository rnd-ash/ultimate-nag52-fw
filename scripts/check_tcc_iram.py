#!/usr/bin/env python3
"""Check direct calls from the linked TCC callback for cache-disabled operation.

This checks the actual ELF, not just IRAM_ATTR annotations. It does not prove
hardware timing, data placement, or the transitive callees' error paths.
"""
import argparse
from pathlib import Path
import re
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path)
    parser.add_argument('--tool-prefix', default=str(Path.home() / '.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32-elf-'))
    args = parser.parse_args()
    symbols = subprocess.check_output([args.tool_prefix + 'nm', '-S', '-C', str(args.elf)], text=True)
    wanted = ('InrushControlSolenoid::on_alarm(', 'inrush_solenoid_timer_isr(')
    found = set()
    for line in symbols.splitlines():
        fields = line.split(maxsplit=3)
        if len(fields) != 4:
            continue
        address, size, kind, name = fields
        target = next((s for s in wanted if name.startswith(s)), None)
        if target is None:
            continue
        found.add(target)
        start = int(address, 16)
        if not 0x40080000 <= start < 0x400A0000:
            raise SystemExit(f'{name}: callback is not in ESP32 IRAM: {address}')
        disasm = subprocess.check_output([args.tool_prefix + 'objdump', '-d', '-C',
            f'--start-address={start}', f'--stop-address={start + int(size, 16)}', str(args.elf)], text=True)
        for instruction in disasm.splitlines():
            if re.search(r'\bcallx\d*\b', instruction):
                raise SystemExit(f'{name}: indirect call needs manual IRAM audit: {instruction}')
            call = re.search(r'\bcall\d*\s+([0-9a-f]+)', instruction)
            if call:
                dest = int(call[1], 16)
                # ESP32 ROM and internal instruction RAM, excluding mapped flash.
                if not 0x40000000 <= dest < 0x400A0000:
                    raise SystemExit(f'{name}: call outside ROM/IRAM: {instruction}')
        print(f'PASS {name}: callback and direct calls in ROM/IRAM')
    if len(found) != len(wanted):
        raise SystemExit(f'Missing callback symbols: {set(wanted) - found}')


if __name__ == '__main__':
    main()
