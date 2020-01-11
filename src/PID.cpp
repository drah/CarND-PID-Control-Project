#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID(bool tune):p_error(0.),i_error(0.),d_error(0.),tune(tune) {}

PID::~PID() {
  if (error_count != 0)
    average_error = sum_error / error_count;
  else
    average_error = 0.;
  fstream out(params_path);
  out << Kp << endl << Ki << endl << Kd << endl;
  out << run_count << endl << error_count << endl << sum_error << endl << average_error << endl;
  out << dp << endl << di << endl << dd << endl;
  out << min_error << endl << state;
  out.close();
}

void PID::Init(string params_path){
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->params_path = params_path;
  fstream in(params_path);
  in >> Kp >> Ki >> Kd;
  in >> run_count >> error_count >> average_error;
  in >> dp >> di >> dd;
  in >> min_error >> state;
  in.close();

  if(state == TWIDDLE_STATE_INIT){
    Kp += dp;
    state = 1;
  }
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;

  run_count += 1;
  if(run_count >= N_ERROR){
    sum_error += cte;
    error_count += 1;
    if(error_count == N_ERROR){
      average_error = sum_error / error_count;
      run_count = 0;
      error_count = 0;
      // twiddle core
      if(average_error < min_error){
        min_error = average_error;
        switch(state){
          case INC_P:
          case DEC_P:
            dp *= 1.1;
            Ki += di;
            state = INC_I;
            break;
          case INC_I:
          case DEC_I:
            di *= 1.1;
            Kd += dd;
            state = INC_D;
            break;
          case INC_D:
          case DEC_D:
            dd *= 1.1;
            Kp += dp;
            state = INC_P;
            break;
          default:
            cout << "state error: " << state << endl;
        }
      }

      else{
        switch(state){
          case INC_P:
            Kp -= 2 * dp;
            state = DEC_P;
            break;
          case DEC_P:
            Kp += dp;
            dp *= 0.9;
            Ki += di;
            state = INC_I;
            break;
          case INC_I:
            Ki -= 2 * di;
            state = DEC_I;
            break;
          case DEC_I:
            Ki += di;
            di *= 0.9;
            Kd += dd;
            state = INC_D;
            break;
          case INC_D:
            Kd -= 2 * dd;
            state = DEC_D;
            break;
          case DEC_D:
            Kd += dd;
            dd *= 0.9;
            Kp += dp;
            state = INC_P;
            break;
          default:
            cout << "state error: " << state << endl;
        }
      }
    }
  }
  cout << "state=" << state << endl;
  cout << "Kp=" << Kp << ", Ki=" << Ki << ", Kd=" << Kd << endl;
  cout << "dp=" << dp << ", di=" << di << ", dd=" << dd << endl;
  cout << "run_count=" << run_count << ", error_count=" << error_count << endl;
  cout << "sum_error=" << sum_error << ", average_error=" << average_error << endl;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error = Kp * p_error + Ki * i_error + Kd * d_error;
  total_error = total_error > 1. ? 1. : total_error < -1. ? -1. : total_error;
  return total_error;
}