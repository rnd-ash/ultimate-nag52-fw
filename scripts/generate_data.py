import os
import sys
import yaml

# Compiler data lengths (ESP32)
L_1_BYTE = ["bool", "uint8_t", "int8_t", "byte", "char"]
L_2_BYTES = ["uint16_t", "int16_t", "short"]
L_4_BYTES = ["uint32_t", "int32_t", "float", "int"]
L_8_BYTES = ["uint64_t", "int64_t", "long"]

ALL_NVS_KEYS=[]

enums = []
i_structs=[]
settings=[]

i_struct_desc=""


def find_setting_idx(name: str) -> int:
    for i in range(0, len(settings)):
        if settings[i].get_name() == name:
            return i
    raise KeyError(name)

output_md = open("MODULE_SETTINGS.md", "w")
output_yml = open("MODULE_SETTINGS.yml", "w")

class Enum:
    def __init__(self, name: str, mappings: [(str, str, int)]) -> None:
        self.name = name.strip();
        self.mappings = mappings[:]
    
    def get_name(self) -> str:
        return self.name
    
    def get_key_value(self, key: str) -> (int, str):
        for i in range(0, len(self.mappings)):
            if self.mappings[i][0] == key:
                return (self.mappings[i][2], self.mappings[i][1])
        raise KeyError(key)


class Variable:
    def __init__(self, data_type, name):
        self.data_type = data_type
        self.name = name
        self.unit = ""
        self.desc = ""

    def set_default_value(self, v):
        self.default = v

    def set_desc(self, desc_str: str):
        self.desc = desc_str.strip()

    def set_unit(self, unit_str: str):
        self.unit = unit
    
    def get_name(self):
        return self.name
    
    def to_markdown_line(self) -> str:
        d = self.desc.replace("\n", " ").strip();
        u = self.unit
        if not self.unit:
            u = "-"
        return "|{}|{}|{}|{}|\n".format(self.name, d, self.data_type, u)
    
    def to_yml_block(self) -> {}:
        d = {}
        d["Name"] = self.name
        d["Description"] = self.desc
        d["Unit"] = self.unit
        d["DataType"] = self.data_type
        d["LengthBytes"] = self.get_length()
        return d

    def get_length(self) -> int: # in BYTES (Everything is aligned)
        if self.data_type in L_1_BYTE:
            return 1
        elif self.data_type in L_2_BYTES:
            return 2
        elif self.data_type in L_4_BYTES:
            return 4
        elif self.data_type in L_8_BYTES:
            return 8
        else:
            # Check enums
            for i in range(0, len(enums)):
                if enums[i].get_name().strip() == self.data_type.strip():
                    return 1 # Enums are always uint8_t type
            for i in range(0, len(i_structs)):
                if i_structs[i].get_name() == self.data_type:
                    return i_structs[i].get_length()
            raise KeyError(self.data_type)



class SettingStructure:
    def __init__(self, name, vars, desc):
        self.name = name
        self.__scn_id__ = 0
        self.variables = vars[:]
        self.desc = desc.strip()
        self.eeprom_key = ""
        for (k, v) in ALL_NVS_KEYS:
            if k.startswith(self.name) and len(self.name) == 3:
                self.eeprom_key = v

    def add_variable(self, var: Variable):
        self.variables.push(var)

    def get_name(self) -> str:
        return self.name
    
    def set_scn_id(self, id: int):
        self.__scn_id__ = id
    
    def get_length(self):
        i = 0
        for e in self.variables:
            i += e.get_length()
        return i
    
    def to_yml_block(self) -> {}:
        d = {}
        d["Name"] = self.name
        d["Description"] = self.desc
        offset = 0
        params=[]
        for var in self.variables:
            res = var.to_yml_block()
            res["OffsetBytes"] = offset
            offset += res["LengthBytes"]
            params.append(res)
        d["Params"] = params
        return d
    
    def to_setting_yml_block(self) -> {}:
        d = self.to_yml_block()
        d["SCN_ID"] = self.__scn_id__
        d["EEPROM_KEY"] = self.eeprom_key
        return d

    def get_variable_index(self, name: str) -> int:
        for v in range(0, len(self.variables)):
            if self.variables[v].get_name() == name:
                return v;
        raise KeyError(name)
    
    def get_heading_name(self) -> str:
        if self.desc:
            return self.desc
        else:
            return "Setting {}".format(self.name)

    def to_markdown_block(self):
        ret = """
## Setting {}

SCN Getter ID: `0x{:02X}`
EEPROM KEY NAME: `{}`
|Setting name|Description|Data Type|Unit|
|:--|:--|:-:|:-:|
""".format(self.name, self.__scn_id__, self.eeprom_key)
        for v in self.variables:
            ret += v.to_markdown_line()

        return ret
    
    def to_markdown_internal_block(self):
        ret = """
## {}\n
|Setting name|Description|Data Type|Unit|
|:--|:--|:-:|:-:|
""".format(self.name)
        for v in self.variables:
            ret += v.to_markdown_line()

        return ret


# Add Linear interp settings to i_structs

lnmin = Variable("float", "new_min")
lnmin.set_desc("Output minimum bound")
lnmax = Variable("float", "new_max")
lnmax.set_desc("Output maximum bound")
lrmin = Variable("float", "raw_min")
lrmin.set_desc("Input clamped minimum")
lrmax = Variable("float", "raw_max")
lrmax.set_desc("Input clamped maximum")

i_structs.append(
    SettingStructure(
        "LinearInterpSetting",
        [
            lnmin,
            lnmax,
            lrmin,
            lrmax
        ],
        ""
    )
)

desc=""


f_in = open("./src/nvs/module_settings.h").readlines()
f_nvs_keys = open("./src/nvs/all_keys.h").readlines()

for line in f_nvs_keys:
    if line.strip().startswith("DO(") and line.endswith("\\\n"):
        key_ty = line.split("DO(")[1].split(",")[0].strip()
        key = line.split("\"")[1]
        if key_ty.endswith("_SETTINGS"):
            ALL_NVS_KEYS.append((key_ty, key))

print(ALL_NVS_KEYS)

vars=[]
enum_maps=[]
in_structure_definition = False
in_structure_default = False
enum_name = ""
unit=""
modifying_structure_idx = -1;

for line in f_in[2:]:

    if not line.strip():
        continue
    
    # Description parsing
    if line.strip().startswith("//"):
        if "UNIT: " in line:
            unit = line.split("UNIT: ")[1].strip()
        else:
            desc += line.strip().removeprefix("//").removesuffix("\n")
        continue

    if line.startswith("#define"):
        # can only be a key
        if "SCN_ID" in line:
            setting_def = line.split("#define ")[1].split("_")[0]
            id = int(line.split(" ")[2], 16)
            settings[find_setting_idx(setting_def)].set_scn_id(id)
        else:
            raise Exception("Invalid line '{}'".format(line))
        
    if line.startswith("typedef struct {"):
        assert(in_structure_definition == False)
        assert(in_structure_definition == False)
        in_structure_definition = True
        i_struct_desc = desc
    elif line.startswith("} __attribute__ ((packed))"):
        in_structure_definition = False
        if line.endswith("MODULE_SETTINGS;\n"):
            name = line.split("((packed)) ")[1].split("_")[0]
            settings.append(SettingStructure(name, vars, i_struct_desc))
        else: # Internal data structure
            name = line.split("((packed)) ")[1].split(";")[0]
            i_structs.append(SettingStructure(name, vars, i_struct_desc))
        vars.clear()
        i_struct_desc = ""
    elif in_structure_definition:
        x = line.strip().removesuffix(";\n").split(" ");
        data_type = x[0];
        variable_name = x[1].removesuffix(";");
        v = Variable(data_type, variable_name);
        v.set_desc(desc)
        v.set_unit(unit)
        vars.append(v)


    if line.startswith("const"):
        assert(in_structure_definition == False)
        assert(in_structure_definition == False)
        in_structure_default = True
        name = line.split("const ")[1].split("_")[0];
        modifying_structure_idx = find_setting_idx(name)
    elif line.startswith("enum "):
        assert(in_structure_definition == False)
        assert(in_structure_definition == False)
        enum_name = line.split("enum ")[1].split(":")[0];
    elif line.startswith("};"):
        in_structure_default = False
        if enum_name != "":
            e = Enum(enum_name, enum_maps)
            enums.append(e)
            enum_maps.clear();
            enum_name = ""
        modifying_structure_idx = -1;

    elif in_structure_default:
        assert(modifying_structure_idx != -1);
        pass
    elif enum_name != "":
        x = line.strip().split("=");
        enum_maps.append((x[0].strip(), desc, int(x[1].strip().removesuffix(","))))

    # Reset description at the END
    if not line.strip().startswith("//"):
        desc = ""
        unit = ""

## Markdown output

output_markdown="# Settings descriptions\n"

for setting in settings:
    output_markdown += setting.to_markdown_block()

output_markdown += "# Enumerations\n"
for e in enums:
    output_markdown += "## {}\n".format(e.name)
    output_markdown += """|Name|Desc|Raw value|
|:-:|:-:|:-:|
"""
    for mapping in e.mappings:
        output_markdown += "|{}|{}|{}|\n".format(mapping[0], mapping[1], mapping[2])


output_markdown += "# Data descriptions for internal structures"

for i in i_structs:
    output_markdown += i.to_markdown_internal_block()
output_md.write(output_markdown)

# YML gen
dict={}

e_list = []
for enum in enums:
    maps={}
    e = {}
    e["Name"] = enum.get_name();
    for x in enum.mappings:
        maps[x[2]] = {"Name":x[0], "Desc":x[1]}
    e["Mappings"] = maps
    e_list.append(e)
dict["Enums"] = e_list


i_struct_list = []

for struct in i_structs:
    i_struct_list.append(struct.to_yml_block())

dict["IStructs"] = i_struct_list

settings_list = []

for struct in settings:
    settings_list.append(struct.to_setting_yml_block())

dict["Settings"] = settings_list

yaml.dump(dict, output_yml, sort_keys=False)

# Compress the YML for embedding in the firmware
f_mod_setting = open("MODULE_SETTINGS.yml", "rb")
f_diag = open("DIAGNOSTICS.yml", "rb")

from zipfile import ZipFile, ZIP_BZIP2

with ZipFile("EMBED.ZIP", "w") as zip:
    zip.compression = ZIP_BZIP2
    zip.write('MODULE_SETTINGS.yml')
    zip.write('DIAGNOSTICS.yml')
zip.close()