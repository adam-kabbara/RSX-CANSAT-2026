#!/usr/bin/env python3
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IN = ROOT / "pfr_analysis" / "flight_data" / "flight_logs.txt"
OUT = ROOT / "pfr_analysis" / "flight_data" / "flight_logs.fixed.txt"

text = IN.read_text(encoding='utf-8')
# normalize line endings
text = text.replace('\r\n', '\n').replace('\r', '\n')
# insert newline before every '1011'
text = re.sub(r'1011', r'\n1011', text)
# remove accidental leading newline
if text.startswith('\n'):
    text = text[1:]
# collapse multiple newlines to a single newline
text = re.sub(r'\n{2,}', '\n', text)

OUT.write_text(text, encoding='utf-8')
print(f"Wrote fixed file: {OUT}")
