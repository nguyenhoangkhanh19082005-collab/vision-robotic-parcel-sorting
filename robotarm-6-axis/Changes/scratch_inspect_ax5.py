import re

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

# Find NAUO (NEXT_ASSEMBLY_USAGE_OCCURRENCE)
nauos = re.findall(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(\s*'([^']*)',\s*'([^']*)',\s*'([^']*)',\s*#(\d+),\s*#(\d+)", text)
print(f"Total NAUOs: {len(nauos)}")

# Map product definitions to product names
# PRODUCT_DEFINITION -> PRODUCT_DEFINITION_FORMATION -> PRODUCT
prod_defs = {}
for m in re.finditer(r"#(\d+)\s*=\s*PRODUCT_DEFINITION\s*\(\s*'[^']*',\s*'[^']*',\s*#(\d+)", text):
    prod_defs[m.group(1)] = m.group(2)

prod_forms = {}
for m in re.finditer(r"#(\d+)\s*=\s*PRODUCT_DEFINITION_FORMATION[^\(]*\(\s*'[^']*',\s*'[^']*',\s*#(\d+)", text):
    prod_forms[m.group(1)] = m.group(2)

prods = {}
for m in re.finditer(r"#(\d+)\s*=\s*PRODUCT\s*\(\s*'([^']+)'", text):
    prods[m.group(1)] = m.group(2)

def get_name(pd_id):
    form_id = prod_defs.get(pd_id)
    if form_id:
        p_id = prod_forms.get(form_id)
        if p_id:
            return prods.get(p_id, f"prod_{p_id}")
    return f"pd_{pd_id}"

for id1, id2, id3, parent_id, child_id in nauos:
    p_name = get_name(parent_id)
    c_name = get_name(child_id)
    if any(k in p_name.lower() or k in c_name.lower() for k in ['ax-4', 'ax-5', 'ax-6', 'riemen', 'scheibe', '17hs', 'pg5', 'nema']):
        print(f"{p_name} -> {c_name} (id: {id1}/{id2})")
