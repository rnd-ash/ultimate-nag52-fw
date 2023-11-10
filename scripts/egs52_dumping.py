import os
import sys
import struct

NUM_MECHANICAL_CALIBRATIONS = 11
# <> v a r
MECHANICAL_START_BYTES = bytes([0x76, 0x61, 0x72])

class MapData:
    def __init__(self, x, y, z, x_unit, y_unit, z_unit):
        self.x = x
        self.y = y
        x_len = len(x)
        y_len = len(y)
        self.z = []
        self.x_unit = x_unit
        self.y_unit = y_unit
        self.z_unit = z_unit

        for i in range(0, y_len-1):
            l = []
            for j in range(0, x_len):
                l.append(z[(i*x_len) + j])
            self.z.append(l.copy())
            l.clear()
        self
    
    def pretty_print(self):
        line="   /*"
        for x in self.x:
            line += "{:4} ".format(x)
        line += "  */"
        print(line)
        for y in range(0, len(self.z)):
            line="        "
            for x in range(0, len(self.z[y])):
                line += "{:4},".format(self.z[y][x])
            line += "  // {}{}".format(self.y[y], self.y_unit)
            print(line)

# 20C 60C 150C (+50 on each value)
# Once this array is found, the X axis of the map is found before 0x0E bytes before
HYDRALIC_MAP_X_INT = [-25, 20, 60, 150]
HYDRALIC_Y_AXIS_BYTES = bytes([0x00, 0x46, 0x00, 0x6E, 0x00, 0xC8])

hydralic_data=[]
mechanical_data=[]

def find_hydralic_map_x(bytes, start_offset) -> (int, list):
    hydralic_map_x = bytes[start_offset:].find(HYDRALIC_Y_AXIS_BYTES)
    l = list()
    if hydralic_map_x != -1:
        print("Found hydralic map X at offset 0x{:02X}".format(hydralic_map_x+start_offset))
        hydralic_map_x += start_offset
        l = list(struct.unpack("<HHHHHHH", bytes[hydralic_map_x-0x0F:hydralic_map_x-1]))
    return (hydralic_map_x, l)

def find_hydralic_map_z(bytes, start_offset) -> list:
    return list(struct.unpack("<HHHHHHHHHHHHHHHHHHHHHHHHHHHH", bytes[start_offset:start_offset+(7*4*2)]))

class MechanicalVariantData:
    def __init__(self, offset, src) -> None:
        bytes_at_offset = src[offset:]
        self.name = struct.unpack("cccc", bytes_at_offset[0:4])
        self.gb_type = int(struct.unpack("B", bytes_at_offset[4:5])[0]) # 1 = small NAG, 0 = Large NAG
        self.unk1 = struct.unpack("B", bytes_at_offset[5:6])
        self.ratios = list(struct.unpack("<HHHHHHHH", bytes_at_offset[6:22]))
        self.something_shift = list(struct.unpack("<HHHHHHHH", bytes_at_offset[22:38]))
        
        clutch_coeffient_z = list(struct.unpack("<HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH", bytes_at_offset[38:134]))

        self.unk2 = list(struct.unpack("<HHHH", bytes_at_offset[134:142]))
        self.unk3 = list(struct.unpack("<HHHH", bytes_at_offset[142:150]))
        self.clutch_release_pressure = list(struct.unpack("<HHHHHH", bytes_at_offset[150:162]))
        self.unk4 = list(struct.unpack("<BBBBBBBB", bytes_at_offset[162:170]))
        self.strongest_loaded_clutch_idx = list(struct.unpack("<BBBBBBBB", bytes_at_offset[170:178]))
        self.unk5 = list(struct.unpack("<HHHHHHHH", bytes_at_offset[178:194]))
        self.unk6 = int(struct.unpack("<H", bytes_at_offset[194:196])[0])
        self.unk7 = int(struct.unpack("<H", bytes_at_offset[196:198])[0])
        self.unk8 = list(struct.unpack("<HHH", bytes_at_offset[198:204]))
        x=[1,2,3,4,5,6]
        y=[0,1,2,3,4,5,6,7]
        x_unit="Clutch"
        y_unit="Gear"
        z_unit="Coef."
        self.clutch_map = MapData(x, y, clutch_coeffient_z, x_unit, y_unit, z_unit)
        self

class HydralicVariantData:
    #
    # NOTES
    # All `shift` arrays seem to indicate 1-2 2-3 3-4 4-5 2-1 3-2 4-3 5-4 order
    #
    def __init__(self, offset, src) -> None:
        bytes_at_offset = src[offset:]
        self.name = struct.unpack("cccc", bytes_at_offset[0:4])
        self.multiplier_r1_1 = struct.unpack("<H", bytes_at_offset[4:6])[0]
        self.multiplier_other_gears = struct.unpack("<H", bytes_at_offset[6:8])[0]
        self.lp_reg_pressure = struct.unpack("<H", bytes_at_offset[8:10])[0]
        self.shift_something0 = list(struct.unpack("<HHHHHHHH", bytes_at_offset[10:26]))
        self.shift_something1 = list(struct.unpack("<HHHHHHHH", bytes_at_offset[26:42]))
        self.shift_something2 = list(struct.unpack("<hhhhhhhh", bytes_at_offset[42:58])) # Signed (Unsigned looks too big)
        self.shift_reg_pressure = struct.unpack("<H", bytes_at_offset[58:60])[0]
        self.shift_factor_multi = list(struct.unpack("<HHHHHHHH", bytes_at_offset[60:76]))
        self.minimum_mpc_pressure = struct.unpack("<H", bytes_at_offset[76:78])[0]
        self.unk0 = struct.unpack("B", bytes_at_offset[78:79])[0]
        self.unk1 = struct.unpack("B", bytes_at_offset[79:80])[0]
        self.unk2 = struct.unpack("<H", bytes_at_offset[80:82])[0]
        self.unk3 = struct.unpack("<H", bytes_at_offset[82:84])[0]
        self.unk4 = struct.unpack("<H", bytes_at_offset[84:86])[0]
        self.inlet_bleeding_factor = struct.unpack("B", bytes_at_offset[86:87])[0]
        self
    
    def add_hydralic_map(self, map):
        self.hydralic_map = map


SCN_OFFSET = 0x4000
HYDRALIC_FIRST_PTR = bytes("mk00", encoding="ASCII")

file_bytes = open(sys.argv[1], "rb").read()


scn_string = file_bytes[SCN_OFFSET:SCN_OFFSET+42]

print("SCN STRING. Name is {}: [{}]".format(str(scn_string[0:4]), ''.join('{:02X} '.format(x) for x in scn_string)))
print("TCC pointer: {}".format(scn_string[4]))
print("Mechanical pointer: {}".format(scn_string[5]))
print("Hydralic pointer: {}".format(scn_string[6]))

hydralic_offset = file_bytes.find(HYDRALIC_FIRST_PTR)
if hydralic_offset != -1:
    for i in range(0,2):
        hydralic_variant = HydralicVariantData(hydralic_offset, file_bytes)
        hydralic_offset += 116 # 0x39610 - 0x3959C
        hydralic_data.append(hydralic_variant)
    offset = 0x30000
    # Need to manually locate this hydralic map
    for i in range(0, 2):
        (offset, axis) = find_hydralic_map_x(file_bytes, offset)
        if offset == -1:
            break
        else:
            data = find_hydralic_map_z(file_bytes, offset + 0x07)
            map = MapData(axis, HYDRALIC_MAP_X_INT, data, "mBar", "C", "mA")
            hydralic_data[i].add_hydralic_map(map)
            offset += 1
    

# Start searching from offset 0x30000
mech_offset = file_bytes[0x30000:].find(MECHANICAL_START_BYTES)
if mech_offset != -1:
    for i in range(0, NUM_MECHANICAL_CALIBRATIONS):
        mech = MechanicalVariantData(0x30000+mech_offset, file_bytes)
        mechanical_data.append(mech)
        mech_offset += 204

print("YOUR DATA (BASED ON SCN):")
print("Mechanical data:")
print(vars(mechanical_data[scn_string[5]]))
print("Friction coefficient map")
mechanical_data[scn_string[5]].clutch_map.pretty_print()
print("HYDRALIC DATA")
print("Hydralic valve body parameters")
print(vars(hydralic_data[scn_string[6]]))
print("Hydralic valve body Solenoid map")
hydralic_data[scn_string[6]].hydralic_map.pretty_print()