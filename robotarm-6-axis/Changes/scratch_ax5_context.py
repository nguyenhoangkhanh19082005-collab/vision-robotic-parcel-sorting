import re

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

# Let's inspect all occurrences of AX-5_1 and AX-5_2 and what components are inside or attached to them
lines = text.splitlines()

for i, line in enumerate(lines):
    if 'AX-5' in line or 'AX-4' in line:
        # print context
        for j in range(max(0, i-5), min(len(lines), i+6)):
            if any(k in lines[j] for k in ['PRODUCT', 'NAUO', 'MAPPED_ITEM', 'AX-5', 'AX-4', '17HS', 'Nema', 'Riemen']):
                print(f"L{j}: {lines[j].strip()[:100]}")
