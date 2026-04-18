#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);

// Blue cartridge (high speed) motors — explicitly set gearset for responsiveness
// Use C++ Motor constructor: port (negative = reversed) and optional gearset
inline pros::Motor intake(21, pros::v5::MotorGears::blue);
inline pros::Motor top_intake(-9, pros::v5::MotorGears::blue);
// Ports were reversed: match_mech uses port H, wing uses port G
inline ez::Piston match_mech('H');
inline ez::Piston wing('G');