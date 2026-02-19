#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor topIntakeMotor(-17); //top intake motor
inline pros::Motor middleIntakeMotor(14); //middle intake motor

inline pros::MotorGroup intake({-17,14}); //intake motor group

inline ez::Piston lilRaaahh('F'); 
inline ez::Piston descore('D'); 
inline ez::Piston middleGoalScore('C'); 
inline ez::Piston middleGoalDescore('B');

inline pros::Optical optical_sensor(8);

inline pros::Task* antiJam = nullptr;

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');