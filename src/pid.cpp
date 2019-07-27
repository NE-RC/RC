#include "../include/pid.h"

PIDController::PIDController(double kP, double kI, double kD, double kF){
  set_gains(kP, kI, kD, kF);
  reset();
}

PIDController::PIDController() : PIDController(0, 0, 0, 0){}

void PIDController::set_gains(double kP, double kI, double kD, double kF){
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->kF = kF;
}

void PIDController::set_target(double target){
  this->target = target;
  has_target = true;
}

double PIDController::update(double current){
  if (!has_target){
    return 0.0;
  }

  double error = target - current;
  double d_error = error - prev_error;
  integral_error += error;

  double output = kP * error + kI * integral_error + kD * d_error + kF * target;
  return output;
}

void PIDController::reset(){
  integral_error = 0;
  prev_error = 0;
  has_target = false;
}
