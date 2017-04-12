#include "PID.h"
#include <chrono>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  prev_cte = 0.0;
  int_cte = 0.0;
  unsigned long timestamp =
    std::chrono::system_clock::now().time_since_epoch() /
    std::chrono::milliseconds(1);

  // Init twiddle
  twiddle = true;
  tw_inc_coef = 1.2;
  tw_dec_coef = 0.9;
  tw_tol = 0.001;
  tw_max_frames = 500;
  tw_runs = 0;
  tw_error = 0;
  tw_max_error = 0;
  tw_frames = 0;
  tw_param_adj = 0;
  tw_next_state = 0; // 0 "NextParam", 1 "IncreasedCheck", 2 "DecreasedCheck"
  params[0] = Kp;
  params[1] = Ki;
  params[2] = Kd;
  d_params[0] = Kp/10;
  d_params[1] = Ki/10;
  d_params[2] = Kd/10;
}

void PID::UpdateError(double cte) {
  if (!twiddle || tw_runs == 0)
  {
    unsigned long timest =
      std::chrono::system_clock::now().time_since_epoch() /
      std::chrono::milliseconds(1);
    long delta_ts = timest - timestamp;
    timestamp = timest;
    double diff_cte = (cte - prev_cte)/delta_ts*1000;
    prev_cte = cte;
    int_cte += cte;
    //steering = -Kp * cte - Kd * diff_cte - Ki * int_cte;
    steering = -(-Kp * cte - Kd * diff_cte - Ki * int_cte);
    // calculate error for initial frames
    tw_frames++;
    tw_max_error += cte*cte;
    if (tw_frames%100 == 0)
    {
      cout << "Inside std, frame:" << tw_frames << " ,cte:" << tw_max_error << endl;
    }
    if (tw_frames > tw_max_frames)
    {
      tw_runs ++;
    }
  }
  else
  {
    if (tw_frames < tw_max_frames) // max_frames run, recalculate parameters
    {
      // increase frame count and error;
      tw_frames++;
      tw_error += cte*cte;
      // calculate steering as per standart aproach
      unsigned long timest =
        std::chrono::system_clock::now().time_since_epoch() /
        std::chrono::milliseconds(1);
      long delta_ts = timest - timestamp;
      timestamp = timest;
      double diff_cte = (cte - prev_cte)/delta_ts*1000;
      prev_cte = cte;
      int_cte += cte;
      steering = -(-Kp * cte - Kd * diff_cte - Ki * int_cte);
      if (tw_frames%100 == 0)
      {
        cout << "Inside twiddle, frame:" << tw_frames << " ,cte:" << tw_error << endl;
      }
    }
    else
    {
      double tol_err = d_params[0]+d_params[1]+d_params[2];
      cout << "Adjusting!!! Tol:" << tol_err << endl;
      // check if tolerance exceeded // End!
      if (tol_err<=tw_tol)
      {
        cout << "Calculations done!!!" << endl;
        cout << "Runs:" << tw_runs << endl;
        cout << "Params Kp:" << params[0] << endl;
        cout << "Params Ki:" << params[1] << endl;
        cout << "Params Kd:" << params[2] << endl;
        steering = 0;
      }
      else
      {
        tw_runs ++;
        switch (tw_next_state)
        {
          case 0:
            tw_param_adj = (tw_param_adj+1)%3;
            tw_frames = 0;
            tw_error = 0;
            params[tw_param_adj] += d_params[tw_param_adj];
            tw_next_state = 1;
            cout << "Next!!! Current Kp:" << params[0] << "Ki:" << params[1] << "Kd:" << params[2] << endl;
            break;
          case 1:
            if (tw_error < tw_max_error)
            {
              tw_max_error = tw_error;
              d_params[tw_param_adj] *= tw_inc_coef;
              tw_next_state = 0;
              cout << "Incr->Next!!! Current Kp:" << params[0] << "Ki:" << params[1] << "Kd:" << params[2] << endl;
            }
            else
            {
              params[tw_param_adj] -= 2*d_params[tw_param_adj];
              tw_frames = 0;
              tw_error = 0;
              tw_next_state = 2;
              cout << "Dec!!! Current Kp:" << params[0] << "Ki:" << params[1] << "Kd:" << params[2] << endl;
            }
            break;
          case 2:
            if (tw_error < tw_max_error)
            {
              tw_max_error = tw_error;
              d_params[tw_param_adj] *= tw_inc_coef;
              tw_next_state = 0;
              cout << "Dec->Next!!! Current Kp:" << params[0] << "Ki:" << params[1] << "Kd:" << params[2] << endl;
            }
            else
            {
              params[tw_param_adj] += d_params[tw_param_adj];
              d_params[tw_param_adj] *= tw_dec_coef;
              tw_next_state = 0;
              cout << "Reset->Next!!! Current Kp:" << params[0] << "Ki:" << params[1] << "Kd:" << params[2] << endl;
            }
            break;
        }
      }
    }
  }
}

double PID::TotalError() {
}

double PID::Steering() {
  return steering;
}
