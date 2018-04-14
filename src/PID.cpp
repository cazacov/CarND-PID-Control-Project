#include "PID.h"
#include <stdio.h>
#include <utility>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, int error_queue_size, int integral_queue_size) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  steps = 0;
  integral_error = 0;
  total_error = 0;
  this->integral_queue_size = integral_queue_size;
  this->error_queue_size = error_queue_size;

  integral_queue = std::queue<double>();
  error_queue = std::queue<double>();
}

void PID::UpdateError(double cte) {

  integral_error += cte;
  integral_queue.push(cte);
  if (integral_queue.size() >= integral_queue_size) {
    integral_error -= integral_queue.front();
    integral_queue.pop();
  }

  total_error += cte*cte;
  error_queue.push(cte*cte);
  if (error_queue.size() >= error_queue_size) {
    total_error -= error_queue.front();
    error_queue.pop();
  }

  double diff_cte = cte - prev_cte;
  prev_cte = cte;
  if (!steps)
  {
    // First step
    diff_cte = 0;
  }

  output = -Kp * cte - Kd * diff_cte - Ki * integral_error;

  if (output > 1) {
    output = 1;
  }
  if (output < -1) {
    output = -1;
  }

  //printf("N: %4d cte=%6.3f steer=%5.3f diff=%5.3f integral_error=%5.2f\n", steps, cte, steer, diff_cte, integral_error);

  steps++;
}

double PID::TotalError() {
  return total_error;
}

double PID::GetOutput() {
  return output;
}

