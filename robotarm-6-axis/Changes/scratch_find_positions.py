import re

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

# Find CONTEXT_DEPENDENT_SHAPE_REPRESENTATION -> REPRESENTATION_RELATIONSHIP
# or AXIS2_PLACEMENT_3D
def find_item_pos(nauo_name):
    # Find NAUO id
    m = re.search(r"#(\d+)\s*=\s*NEXT_ASSEMBLY_USAGE_OCCURRENCE\s*\(\s*'" + nauo_name + r"'", text)
    if not m:
        return None
    nauo_id = m.group(1)
    
    # Find CDSR
    m2 = re.search(r"#(\d+)\s*=\s*CONTEXT_DEPENDENT_SHAPE_REPRESENTATION\s*\(\s*#\d+,\s*#(\d+)\s*\)", text)
    # Search for PRODUCT_DEFINITION_SHAPE referencing this NAUO
    m_pds = re.search(r"#(\d+)\s*=\s*PRODUCT_DEFINITION_SHAPE[^\(]*\([^\)]*#" + nauo_id + r"\s*\)", text)
    if m_pds:
        pds_id = m_pds.group(1)
        m_cdsr = re.search(r"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION\s*\(\s*#" + pds_id + r",\s*#(\d+)\s*\)", text)
        if m_cdsr:
            rr_id = m_cdsr.group(1)
            # Find ITEM_DEFINED_TRANSFORMATION
            m_rr = re.search(r"#" + rr_id + r"\s*=\s*REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION[^\(]*\([^\)]*#(\d+)\s*\)", text)
            if m_rr:
                idt_id = m_rr.group(1)
                m_idt = re.search(r"#" + idt_id + r"\s*=\s*ITEM_DEFINED_TRANSFORMATION[^\(]*\([^\)]*#(\d+),\s*#(\d+)\s*\)", text)
                if m_idt:
                    ax2 = m_idt.group(2)
                    m_ax = re.search(r"#" + ax2 + r"\s*=\s*AXIS2_PLACEMENT_3D[^\(]*\([^\)]*#(\d+)", text)
                    if m_ax:
                        pt_id = m_ax.group(1)
                        m_pt = re.search(r"#" + pt_id + r"\s*=\s*CARTESIAN_POINT[^\(]*\(\s*'[^']*',\s*\(\s*([^,\)]+),\s*([^,\)]+),\s*([^,\)]+)\s*\)", text)
                        if m_pt:
                            return [float(m_pt.group(1)), float(m_pt.group(2)), float(m_pt.group(3))]
    return None

for nauo in ['NAUO21', 'NAUO42', 'NAUO81', 'NAUO123', 'NAUO156', 'NAUO167', 'NAUO38', 'NAUO39', 'NAUO120', 'NAUO140', 'NAUO161', 'NAUO170', 'NAUO192', 'NAUO193']:
    pos = find_item_pos(nauo)
    print(f"{nauo}: {pos}")
