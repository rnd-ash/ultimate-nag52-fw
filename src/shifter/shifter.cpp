#include "shifter.h"

Shifter *shifter = nullptr;

void Shifter::set_brake_is_pressed(bool is_pressed)
{
	is_brake_pressed = is_pressed;
}

void Shifter::set_vehicle_speed(uint16_t front_left, uint16_t front_right)
{
	if ((UINT16_MAX != front_left) && (UINT16_MAX != front_right))
	{
		vVeh = ((float)(((front_left + front_right) / 2) * ((int32_t)(VEHICLE_CONFIG.wheel_circumference)) * 6)) / 100000.F;
	}
}
