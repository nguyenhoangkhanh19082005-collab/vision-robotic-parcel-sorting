import re

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

# Let's find cylindrical surfaces or circles for Riemenscheibe 40mm and 16mm
for name in ['Riemenscheibe 16mm', 'Riemenscheibe 40mm', 'Riemenscheibe 55mm']:
    print(f"\n--- Searching for {name} ---")
    pos = text.find(f"'{name}'")
    if pos != -1:
        snippet = text[pos-200:pos+1000]
        print(snippet[:300])
