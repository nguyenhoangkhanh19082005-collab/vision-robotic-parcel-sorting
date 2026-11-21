import re
import numpy as np

with open('Roboter.STEP', 'r', errors='ignore') as f:
    text = f.read()

# Let's find all occurrences of motors and their CARTESIAN_POINT coordinates
# NAUO for motors
# NAUO21: Nema17 Schrittmotor
# NAUO42: Nema17 Schrittmotor
# NAUO81: STEPPERONLINE.17HS13-0404S-PG5
# NAUO123: STEPPERONLINE.17HS13-0404S-PG5
# NAUO156: STEPPERONLINE.17HS13-0404S-PG5
# NAUO167: Nema17 Schrittmotor

# Notice: There are EXACTLY 6 motors:
# 3 x STEPPERONLINE.17HS13-0404S-PG5 (Planetary gearbox 5.18:1)
# 3 x Nema17 Schrittmotor (Standard 1:1)
print("Motor breakdown in CAD:")
print("- 3 x 17HS13-0404S-PG5 (Planetary 5.18:1)")
print("- 3 x Nema17 Schrittmotor (Standard 1:1)")
