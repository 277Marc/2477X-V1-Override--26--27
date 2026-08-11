#pragma once

#include "EZ-Template/PID.hpp"
#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');


//inline pros::Controller master(pros::E_CONTROLLER_MASTER);
//inline pros::Controller partner(pros::E_CONTROLLER_PARTNER);

inline ez::Piston Claw('H');


inline pros::Motor rARM(17);
inline pros::Motor lARM(6);

inline pros::Motor LiftL(7);
inline pros::Motor LiftR(-8);

//////////////////////////////////////////////////////////////////////////////////////////////////

inline pros::Motor Wrist(8);

inline void setARM(int input){
    // Reverse wrist motor direction to match sensor sign/direction
    lARM.move(-input);
    rARM.move(input);
}

// Wrist PID tuned for smoother movement and reduced jitter
// Reduced proportional gain and increased damping to cut overshoot and improve target stability.
inline ez::PID armPID(0.12, 0.003, 0.010, 0, "ArmD");

inline void armWait(){
    while(armPID.exit_condition({lARM, rARM}, true) == ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////



inline void setLift(int input){
    // Motor directions are reversed for this lift setup, so invert the command.
    LiftL.move(input);
    LiftR.move(input);
}

// PID tuned for rotation sensor centi-degrees units — using stronger gains for full lift power
// Increased kP and kD to help the lift overcome gravity and move more decisively
inline ez::PID liftPID(0.15, 0.004, 0.008, 0, "LiftD");

// Custom autonomous functions for DR4B lift
inline void liftWait(){
    while(liftPID.exit_condition({LiftL, LiftR}, true) == ez::RUNNING){
        pros::delay(ez::util::DELAY_TIME);
    }
}