#ifndef __PID_H_
#define __PID_H_

enum PidTpyeDef
{
  LLAST = 0,
  LAST,
  NOW,
  POSITON_PID,
  DELTA_PID,
};

class PID
{
public:
  PID(PidTpyeDef pid_mode, double kp, double ki, double kd, uint32_t output_max,uint32_t input_max_err, uint32_t integral_limit, double output_deadband);
  void resetPid(double kp, double ki, double kd);
  double calcPid(double set, double get);

private:
  PidTpyeDef pid_mode_;

  uint32_t output_max_;
  uint32_t integral_limit_;

  double kp_;
  double ki_;
  double kd_;

  double set_;
  double get_;
  double err_[3];

  double pout_;
  double iout_;
  double dout_;
  double out_;

  double input_max_err_;   //input max err;
  double output_deadband_; //output deadband;

  void adsLimit(double *a, double abs_max);
};

#endif // __PID_H_
