# Converts my text files from MBUX-PORT project to C++ header files to use with my ECU!

import os
import sys

input_file=open(sys.argv[1])
output_dir=sys.argv[2]

print("Input file: ", input_file, " Output dir: ", output_dir)

ecu=""

for line in input_file:
    if "FRAME " in line:
        print(line)
        parsed = line.replace(" ", "").split("FRAME")[1]
        ecu = parsed.split(" (")[0].split("_")[0]
        can_id = parsed.split("(")[1].split(")")[0]
        print("Frame {} for ECU {}".format(can_id, ecu))