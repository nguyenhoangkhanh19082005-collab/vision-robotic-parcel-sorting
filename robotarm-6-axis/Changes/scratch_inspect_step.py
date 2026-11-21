import re

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

matches = set(re.findall(r"PRODUCT\s*\(\s*'([^']+)'", text))
print(f"Products found: {len(matches)}")
for m in sorted(matches):
    print(" -", m)
