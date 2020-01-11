#ifndef PID_H
#define PID_H
#include <string>
#include <fstream>
#include <iostream>
using std::fstream;
using std::string;
using std::endl;
using std::cout;
#define N_ERROR (100)

class PID {
 public:
  /**
   * Constructor
   */
  PID(bool);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(string params_path);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  /* twiddle */
  bool tune;
  string params_path;
  int run_count;
  int error_count;
  double sum_error;
  double average_error;
  double dp;
  double di;
  double dd;
  double min_error;
  double best_Kp;
  double best_Ki;
  double best_Kd;
  int state;
  enum State {
    TWIDDLE_STATE_INIT,
    INC_P,
    DEC_P,
    INC_I,
    DEC_I,
    INC_D,
    DEC_D
  };
};

#endif  // PID_H
