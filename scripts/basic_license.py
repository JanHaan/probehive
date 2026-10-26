"""Keep only the Vivado Basic tier of an AMD license file.

    python3 scripts/basic_license.py ~/Downloads/Xilinx.lic ~/.Xilinx/Xilinx.lic

AMD's portal issues one file with every tier on it (Basic, Alveo, an Enterprise evaluation),
and Vivado then picks the Alveo tier, which offers no Artix-7 part.  Each FlexLM record is
signed on its own, so the Basic records and their PACKAGE definition stand alone."""
import re, sys

src = open(sys.argv[1]).read()
records = re.findall(r"^(?:INCREMENT|FEATURE|PACKAGE)\b(?:[^\n]*\\\n)*[^\n]*\n", src, re.M)
keep = [r for r in records if "License_Tier:BASIC" in r]
names = {r.split()[1] for r in keep}
keep += [r for r in records if r.startswith("PACKAGE") and r.split()[1] in names]
if not any(r.startswith("PACKAGE") for r in keep):
    sys.exit(f"{sys.argv[1]}: no Basic tier package in it")
with open(sys.argv[2], "w") as f:
    f.write("".join(keep))
print(f"{sys.argv[2]}: {len(keep)} of {len(records)} records, the Basic tier's")
