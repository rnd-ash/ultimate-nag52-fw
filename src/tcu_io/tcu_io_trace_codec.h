#ifndef TCU_IO_TRACE_CODEC_H
#define TCU_IO_TRACE_CODEC_H

#include <stddef.h>

#include "tcu_io_data_source.h"

namespace TCUIO {

size_t format_hardware_frame_csv_header(char* out, size_t out_size);
size_t format_hardware_frame_csv(const TcuIoHardwareFrame& frame, char* out, size_t out_size);
bool parse_hardware_frame_csv(const char* line, TcuIoHardwareFrame* out);

size_t format_actuator_frame_csv_header(char* out, size_t out_size);
size_t format_actuator_frame_csv(const TcuIoActuatorFrame& frame, char* out, size_t out_size);
bool parse_actuator_frame_csv(const char* line, TcuIoActuatorFrame* out);

} // namespace TCUIO

#endif