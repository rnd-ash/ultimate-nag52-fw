#
# Converts the can_data.txt files into C++ headers for this project!
#
# This program takes 2 arguments:
# 1. Input can_data.txt file
# 2. Output directory for header files
# 3. Optional - Global #ifdef guard for file
#
#
# IMPORTANT: The output bitfield structs are in LITTLE ENDIAN.
# The original ECUs use BIG Endian, so the bit ordering is flipped!

import os
import sys


input_file=open(sys.argv[1], 'r')
output_dir=sys.argv[2]

output_guard = False
global_guard=""
if len(sys.argv) > 3:
    global_guard=sys.argv[3]
    output_guard = True

def clear_bit(mask, bit):
    return mask & ~(1<<bit)

def remove_useless_from_line(l: str) -> str:
    return l.strip().replace("\n", "")

class EnumEntry:
    def __init__(self, name: str, raw: int, desc: str):
        self.name = name
        self.raw = raw
        self.desc = desc

class Signal:
    def __init__(self, name: str, desc: str, length: int, offset: int, mask_bin: str, unit: str):
        self.name = name
        self.desc = desc
        self.length = length
        self.offset = offset
        self.is_iso_tp = False
        self.is_number = False
        self.is_bool = False
        self.is_char = False
        self.number_data = (0.0, 0.0) # Multiplier, offset
        self.is_enum = True
        self.unit = unit
        self.mask_bin = mask_bin
        self.enum_table = []

    def is_masked_enum(self) -> bool:
        return self.mask_bin

    def get_return_data_type(self, frame_name: str) -> str:
        if self.is_number or len(self.mask_bin) != 0:
            if self.length <= 8: # 8 bit wide data
                return "uint8_t"
            elif self.length <= 16: # 16 bit wide data
                return "uint16_t"
            elif self.length <= 32: # 32 bit wide data
                return "uint32_t"
            else:
                return "uint64_t" # 64 bit wide data
        elif self.is_bool: # Boolean data type
            return "bool"
        elif self.is_enum: # Enum data type
            return "{}_{}{}".format(frame_name, self.name, global_guard)
        elif self.is_char:
            return "char"
        else:
            return"" # ??

    def get_masked_data_type(self, frame_name: str) -> str:
        if self.is_number:
            if self.length <= 8: # 8 bit wide data
                return "uint8_t"
            elif self.length <= 16: # 16 bit wide data
                return "uint16_t"
            elif self.length <= 32: # 32 bit wide data
                return "uint32_t"
            else:
                return "uint64_t" # 64 bit wide data
        elif self.is_bool: # Boolean data type
            return "bool"
        elif self.is_enum: # Enum data type
            return "{}_{}{}".format(frame_name, self.name, global_guard)
        elif self.is_char:
            return "char"
        else:
            return"" # ??

    def get_entry(self, frame_name: str) ->str:
        return ""

    def add_enum(self, e: EnumEntry):
        self.enum_table.append(e)

    def add_data_str(self, dt: str):
        self.is_bool = False
        self.is_enum = False
        self.is_iso_tp = False
        self.is_number = False

        if dt.strip() == "ISO_TP":
            self.is_iso_tp = True
        elif dt.strip() == "CHAR":
            self.is_char = True
        elif dt.strip() == "BOOL":
            self.is_bool = True
        elif "ENUM" in dt:
            self.is_enum = True
        elif "NUMBER" in dt:
            self.is_number = True
            multiplier = float(dt.split("_MULTIPLIER_: ")[1].split(",")[0])
            offset = float(dt.split("_OFFSET_: ")[1].split(")")[0])
            self.number_data = (multiplier, offset)
        else:
            print(dt)

class Frame:
    def __init__(self, name: str, id: int):
        self.name = name
        self.can_id = id
        self.signals=[]
    
    def add_signal(self, s: Signal):
        self.signals.append(s)

class ECU:
    def __init__(self, name: str):
        self.name = name
        self.frames=[]

    def add_frame(self, f: Frame):
        self.frames.append(f)

    def filter_frames(self):
        cloned = self.frames.copy()
        self.frames.clear()
        for x in cloned:
            if len(x.signals) > 1:
                self.frames.append(x)

    def make_output_str(self) -> str:
        self.filter_frames()
        # Create output header string
        tmp = """
/**
* AUTOGENERATED BY convert.py
* DO NOT EDIT THIS FILE!
*
* IF MODIFICATIONS NEED TO BE MADE, MODIFY can_data.txt!
*
* CAN Defintiion for ECU '{0}'
*/

#ifndef __ECU_{0}_H_
#define __ECU_{0}_H_

#include <stdint.h>
    """.format(self.name)

        for f in self.frames:
            tmp += "\n#define {}{}_CAN_ID 0x{:04X}".format(f.name.strip().removesuffix("h"), global_guard, f.can_id)

        tmp += "\n\n"
        # Now iterate over all enums of the ECU
        for x in self.frames:
            for s in x.signals:
                if s.is_enum:
                    tmp += "/** {} */".format(s.desc)
                    # Max enum value
                    vtype = "uint8_t"
                    if s.length <= 8:
                        vtype = "uint8_t"
                    if s.length <= 16:
                        vtype = "uint16_t"
                    elif s.length <= 32:
                        vtype = "uint32_t"
                    tmp += "\nenum class {}_{}{} : {} {{".format(x.name, s.name, global_guard, vtype)
                    for e in s.enum_table:
                        tmp += "\n\t{} = {}, // {}".format(e.name, e.raw, e.desc)
                    tmp += "\n};\n\n"

        # Now create our type unions for CAN Frames!
        for x in self.frames:
            struct_name = x.name.strip().removesuffix("h")
            tmp += "\n\ntypedef union {" # Struct name
            tmp += "\n\tuint64_t raw;" # Store raw value
            tmp += "\n\tuint8_t bytes[8];"
            tmp += "\n\tstruct {"
            # Now add all the signals!
            curr_offset=64
            # Reverse for Big -> Little conversion
            sig_rev = x.signals
            sig_rev.reverse()
            padding_idx = 1
            masked_signals = []
            for s in sig_rev:
                curr_offset -= (s.length)
                if curr_offset > s.offset:
                    padding_len=curr_offset-s.offset
                    want_data = "uint64_t"
                    if padding_len == 1:
                        want_data = "bool"
                    elif padding_len <= 8:
                        want_data = "uint8_t"
                    elif padding_len <= 16:
                        want_data = "uint16_t"
                    elif padding_len <= 32:
                        want_data = "uint32_t"
                    tmp += "\n\t\t /** BITFIELD PADDING. DO NOT CHANGE **/"
                    tmp += "\n\t\t{} __PADDING{}__: {};".format(want_data, padding_idx, padding_len)
                    curr_offset = s.offset
                    padding_idx += 1
                if s.is_masked_enum():
                    tmp += "\n\t\t/** !!MASKED SIGNAL!! Use get_{}() and set_{}() to use this data! **/".format(s.name, s.name)
                    tmp += "\n\t\t{} {}: {};".format(s.get_return_data_type(x.name), s.name, s.length)
                    masked_signals.append(s)
                else:
                    tmp += "\n\t\t/** {} **/".format(s.desc)
                    tmp += "\n\t\t{} {}: {};".format(s.get_return_data_type(x.name), s.name, s.length)

            if curr_offset > 0:
                want_data = "uint64_t"
                if curr_offset == 1:
                    want_data = "bool"
                elif curr_offset <= 8:
                    want_data = "uint8_t"
                elif curr_offset <= 16:
                    want_data = "uint16_t"
                elif curr_offset <= 32:
                    want_data = "uint32_t"
                tmp += "\n\t\t /** BITFIELD PADDING. DO NOT CHANGE **/"
                tmp += "\n\t\t{} __PADDING{}__: {};".format(want_data, padding_idx, curr_offset)
            tmp += "\n\t} __attribute__((packed));"
            tmp += "\n\t/** Gets CAN ID of {}{} **/".format(struct_name, global_guard)
            tmp += "\n\tuint32_t get_canid(){{ return {}{}_CAN_ID; }}".format(struct_name, global_guard)
            
            for s in masked_signals:
                tmp += "\n\t/** Gets {} **/".format(s.desc)
                tmp += "\n\t{0} get_{1}(){{ return ({0})(raw >> {2} & {3}); }}".format(s.get_masked_data_type(x.name), s.name, 64-(s.offset+s.length), s.mask_bin)

                tmp += "\n\t/** Sets {} **/".format(s.desc)


                # 2 - Set mask
                # 3 - Value & mask
                # 4 - shift
                mask = 0xFFFFFFFFFFFFFFFF
                for bit in range (0, len(s.mask_bin)):
                    if s.mask_bin[len(s.mask_bin)-bit-1] == '1':
                        mask = clear_bit(mask, (64-s.offset+bit-s.length))
                tmp += "\n\tvoid set_{1}({0} v){{ raw = (raw & 0x{2:{fill}16x}) | ((uint64_t)v & 0b{3}) << {4}; }}".format(s.get_masked_data_type(x.name), s.name, mask, s.mask_bin, 64-(s.offset+s.length), fill='0')
            # Setters and getters!
            
            
            tmp += "\n}} {}{};\n\n".format(struct_name, global_guard)


        # Now magic to create the class ;)

        num_frames = len(self.frames)

        tmp += "\n\nclass ECU_{} {{".format(self.name)
        tmp += "\n\tpublic:"
        # Setter function to import supported frames to the ECU
        tmp += """
        /**
         * @brief Imports the CAN frame given the CAN ID, CAN Contents, and current timestamp
         *
         * Returns true if the frame was imported successfully, and false if import failed (Due to non-matching CAN ID).
         *
         * NOTE: The endianness of the value cannot be guaranteed. It is up to the caller to correct the byte order!
         */
        bool import_frames(uint64_t value, uint32_t can_id, uint32_t timestamp_now) {
            uint8_t idx = 0;
            bool add = true;
            switch(can_id) {"""
        for idx, frame in enumerate(self.frames):
            tmp += """
                case {0}{2}_CAN_ID:
                    idx = {1};
                    break;""".format(frame.name.strip().removesuffix("h"), idx, global_guard)
        tmp += """
                default:
                    add = false;
                    break;
            }
            if (add) {
                LAST_FRAME_TIMES[idx] = timestamp_now;
                FRAME_DATA[idx] = value;
            }
            return add;
        }
        """
        # Now do getters!
        for idx, frame in enumerate(self.frames):
            tmp += """
        /** Sets data in pointer to {0}
          * 
          * If this function returns false, then the CAN Frame is invalid or has not been seen
          * on the CANBUS network yet. Meaning it's data cannot be used.
          *
          * If the function returns true, then the pointer to 'dest' has been updated with the new CAN data
          */
        bool get_{0}(const uint32_t now, const uint32_t max_expire_time, {0}{2}* dest) const {{
            bool ret = false;
            if (dest != nullptr && LAST_FRAME_TIMES[{1}] <= now && now - LAST_FRAME_TIMES[{1}] < max_expire_time) {{
                dest->raw = FRAME_DATA[{1}];
                ret = true;
            }}
            return ret;
        }}
            """.format(frame.name.strip().removesuffix("h"), idx, global_guard)
        tmp += "\n\tprivate:"
        tmp += "\n\t\tuint64_t FRAME_DATA[{0}];".format(num_frames)
        tmp += "\n\t\tuint32_t LAST_FRAME_TIMES[{0}];".format(num_frames)
        tmp += "\n};"

        # Lastly append endif guard
        tmp += "\n#endif // __ECU_{}_H_".format(self.name)
        return tmp


current_ecu: ECU = None
current_frame: Frame = None
current_signal: Signal = None


for line in input_file:
    #print(line)
    l = remove_useless_from_line(line)
    if not l.startswith("#"): # Ignore comments
        if l.startswith("ECU "):
            ecu = l.split("ECU ")[1].strip()
            print("ECU "+ecu)
            if getattr(current_ecu, 'name', 'nan') != ecu and current_ecu != None:
                # Check if frame / signal is none
                if current_signal and current_frame:
                    if len(current_frame.signals) > 0: # Ignore ISO-TP endpoints
                        current_frame.add_signal(current_signal)
                    current_signal = None
                if current_frame and current_ecu:
                    current_ecu.add_frame(current_frame)
                    current_frame = None
                open("{}/{}.h".format(output_dir, current_ecu.name), 'w').write(current_ecu.make_output_str()) # Write tmp output str to file
            current_ecu = ECU(ecu)
        elif l.startswith("FRAME"):
            frame_name = l.split("FRAME ")[1].split("(")[0].strip()
            frame_id = int(l.split("(")[1].split(")")[0], 0)
            print("FRAME "+frame_name)
            if current_signal and current_frame:
                current_frame.add_signal(current_signal)
            if current_frame:
                if len(current_frame.signals) > 0: # Ignore ISO-TP endpoints
                    # Its a new frame
                    current_ecu.add_frame(current_frame)
            current_frame = Frame(frame_name, frame_id)
            current_signal = None
        elif l.startswith("SIGNAL"):
            if current_signal:
                current_frame.add_signal(current_signal)
            signal_mask = ""
            signal_name = l.split("SIGNAL ")[1].split(", ")[0].strip()
            signal_offset = int(l.split("OFFSET: ")[1].split(",")[0], 10)
            signal_length = int(l.split("LEN: ")[1].split(",")[0], 10)
            if "MASK: " in line:
                signal_mask = l.split("MASK: ")[1].split(", ")[0].strip()
            signal_desc = l.split("DESC: ")[1].split(", DATA TYPE")[0].strip()
            try:
                signal_dt = l.split(", DATA TYPE ")[1]
            except Exception as e:
                signal_dt = "RAW"
            unit=""
            if "UNIT: " in l:
                unit = l.split("UNIT: ")[1]
            current_signal = Signal(signal_name, signal_desc, signal_length, signal_offset, signal_mask, unit)
            current_signal.add_data_str(signal_dt)
        elif l.startswith("ENUM"):
            if current_signal:
                enum_name = l.split("ENUM ")[1].split(", ")[0].strip()
                enum_raw = int(l.split("RAW: ")[1].split(", ")[0])
                enum_desc = l.split("DESC: ")[1].strip()
                current_signal.add_enum(EnumEntry(enum_name, enum_raw, enum_desc))


if current_signal and current_frame:
    current_frame.add_signal(current_signal)
    current_signal = None
if current_frame and current_ecu:
    current_ecu.add_frame(current_frame)
    current_frame = None
# Write last ECU in DB
open("{}/{}.h".format(output_dir, current_ecu.name), 'w').write(current_ecu.make_output_str()) # Write tmp output str to file
