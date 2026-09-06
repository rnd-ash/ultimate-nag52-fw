#include "shifter.h"

void Shifter::set_brake_is_pressed(bool is_pressed)
{
	is_brake_pressed = is_pressed;
}

void Shifter::set_vehicle_speed(wheel_rpm_2x_t front_left, wheel_rpm_2x_t front_right)
{
	if (WheelSpeed::is_valid(front_left) && WheelSpeed::is_valid(front_right))
	{
		// The shift is by TWO, not one: both inputs are doubled wheel RPM, so
		// this is the mean real RPM of the axle. WheelSpeed::mean_rpm_u16 says so.
		vVeh = ((float)((int32_t)WheelSpeed::mean_rpm_u16(front_left, front_right) * ((int32_t)(vehicle_config->wheel_circumference)) * 6)) / 100000.F;
	}
}
