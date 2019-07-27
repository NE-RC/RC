#include "pid.h"

#include <algorithm>

PIDController::PIDController(double kP, double kI, double kD, double kF){
  SetGains(kP, kI, kD, kF);
  SetOutputBounds(-1.0, 1.0);
  SetTolerance(0.1);
  Reset();
}

PIDController::PIDController() : PIDController(0, 0, 0, 0){}

void PIDController::SetGains(double kP, double kI, double kD, double kF){
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->kF = kF;
}

void PIDController::SetTarget(double target){
  this->target = target;
  has_target = true;
  at_target = false;
}

void PIDController::SetOutputBounds(double min, double max){
  min_output = min;
  max_output = max;
}

void PIDController::SetTolerance(double tolerance){
  this->tolerance = tolerance;
}

double PIDController::Update(double current){
  if (!has_target){
    return 0.0;
  }

  double error = target - current;
  
  if (error < tolerance){
    at_target = true;
  }
  else{
    at_target = false;
  }

  double d_error = error - prev_error;
  integral_error += error;

  double output = kP * error + kI * integral_error + kD * d_error + kF * target;
  output = std::min(std::max(output, min_output), max_output);

  prev_error = error;
  return output;
}

void PIDController::Reset(){
  integral_error = 0;
  prev_error = 0;
  has_target = false;
}

bool PIDController::Done() const {
  if (!has_target){
    return false;
  }
  return at_target;
}
