#ifndef PID_H
#define PID_H

#include <vector>

class PID {
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

  double prev_cte;
  double int_cte;
  double steering;

  unsigned long timestamp;

  bool twiddle;
  double tw_error;
  double tw_max_error;
  float tw_inc_coef;
  float tw_dec_coef;
  float tw_tol;
  unsigned int tw_frames;
  unsigned int tw_max_frames;
  unsigned int tw_runs;
  float params[3];
  float d_params[3];
  unsigned int tw_param_adj;
  unsigned int tw_next_state;
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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the steering angle.
  */
  double Steering();
};

#endif /* PID_H */
