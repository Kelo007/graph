import re
import os

filename="solution.h"

def replace(W):
  with open(filename,'r',encoding="utf-8") as f:
    newlines = []
    for line in f.readlines():
      if line.strip().startswith("double W = "):
        line = "  double W = %f;\n" % W
      newlines.append(line)
  with open(filename, 'w') as f:
    for line in newlines:
      f.write(line)

W = [1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 2.0, 2.2, 2.4, 2.6, 3.0]

for w in W:
  replace(w)
  os.system("zip %.2f.zip solution.h solution.cpp" % w)

zipfiles = ["%.2f.zip" % w for w in W]

os.system("rm all.zip")
os.system("zip all.zip %s" % " ".join(zipfiles))
os.system("rm %s" % " ".join(zipfiles))
