#include "shifter.h"

void Shifter::set_brake_is_pressed(bool is_pressed)
{
	is_brake_pressed = is_pressed;
}

void Shifter::set_vehicle_speed(WheelData front_left, WheelData front_right)
{
	if ((WheelDirection::Forward == front_left.current_dir) && (WheelDirection::Forward == front_right.current_dir))
	{
		vVeh = ((float)(((front_left.double_rpm + front_right.double_rpm) >> 2) * ((int32_t)(vehicle_config->wheel_circumference)) * 6)) / 100000.F;
	}
}
