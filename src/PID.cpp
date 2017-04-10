#include "PID.h"
#include <chrono>

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
}

void PID::UpdateError(double cte) {
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
}

double PID::TotalError() {
}

double PID::Steering() {
  return steering;
}
