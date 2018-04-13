#include "PID.h"
#include <stdio.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  steps = 0;
  integral_error = 0;
}

void PID::UpdateError(double cte) {
  integral_error += cte;

  double diff_cte = cte - prev_cte;
  prev_cte = cte;
  if (!steps)
  {
    // First step
    diff_cte = 0;
  }

  steer = -Kp * cte - Kd * diff_cte - Ki * integral_error;

  if (steer > 1) {
    steer = 1;
  }
  if (steer < -1) {
    steer = -1;
  }

  //printf("N: %4d cte=%6.3f steer=%5.3f diff=%5.3f integral_error=%5.2f\n", steps, cte, steer, diff_cte, integral_error);

  steps++;
}

double PID::TotalError() {
}

double PID::GetSteer() {
  return steer;
}

