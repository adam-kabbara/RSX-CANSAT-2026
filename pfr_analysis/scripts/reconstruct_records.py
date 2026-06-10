#!/usr/bin/env python3
"""
Reconstruct records in a log file by joining fragments between '1011,' markers.

Usage:
  python3 scripts/reconstruct_records.py input.txt output.txt

This treats every occurrence of the substring '1011,' as the start of a new
record and collects everything up to the next '1011,' into one line (removing
internal newlines). Useful when logs were split across lines.
"""
import sys


def reconstruct(inp, outp):
    data = open(inp, 'r', encoding='utf-8', errors='ignore').read()
    # find positions of '1011,'
    starts = []
    idx = data.find('1011,')
    while idx != -1:
        starts.append(idx)
        idx = data.find('1011,', idx+1)
    if not starts:
        print('No records starting with 1011, found; writing original')
        open(outp, 'w', encoding='utf-8').write(data)
        return
    records = []
    for i, s in enumerate(starts):
        e = starts[i+1] if i+1 < len(starts) else len(data)
        rec = data[s:e]
        # remove newlines inside the record and strip
        rec = rec.replace('\n', ' ').replace('\r', '')
        rec = rec.strip()
        # ensure it begins with 1011,
        if not rec.startswith('1011,'):
            continue
        records.append(rec)
    with open(outp, 'w', encoding='utf-8') as f:
        for r in records:
            f.write(r + '\n')
    print(f'Wrote {len(records)} reconstructed records to {outp}')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: reconstruct_records.py input.txt output.txt')
        sys.exit(1)
    reconstruct(sys.argv[1], sys.argv[2])
