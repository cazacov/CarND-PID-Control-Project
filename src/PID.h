#ifndef PID_H
#define PID_H

#include <vector>
#include <queue>

class PID {
private:
  int steps;
  double integral_error;
  double total_error;
  double prev_cte;
  double output;
  int integral_queue_size;
  int error_queue_size;
  std::queue<double>  integral_queue;
  std::queue<double>  error_queue;
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, int error_queue_size, int integral_queue_size = 200);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Returns steering angle
   */
  double GetOutput();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
