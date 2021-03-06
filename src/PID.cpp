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

  double diff_cte = cte - prev_cte;
  prev_cte = cte;
  if (!steps)
  {
    // First step
    diff_cte = 0;
  }

  integral_error += cte;
  integral_queue.push(cte);
  if (integral_queue.size() >= integral_queue_size) {
    integral_error -= integral_queue.front();
    integral_queue.pop();
  }

  double loss = cte*cte;              // Penalize leaving middle of the row
  loss += 10 * diff_cte * diff_cte;   // Penalize intensive steering

  total_error += loss;
  error_queue.push(loss);
  if (error_queue.size() >= error_queue_size) {
    total_error -= error_queue.front();
    error_queue.pop();
  }


  // PID controller
  output = -Kp * cte - Kd * diff_cte - Ki * integral_error;

  if (output > 1) {
    output = 1;
  }
  if (output < -1) {
    output = -1;
  }

  steps++;
}

double PID::TotalError() {
  return total_error;
}

double PID::GetOutput() {
  return output;
}

