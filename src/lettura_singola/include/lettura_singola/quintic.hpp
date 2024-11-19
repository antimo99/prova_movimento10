#include <math.h> 

double quintic(double t, double qi, double qf, double tf)
{
 if (t <= 0.0)
  {
    return 0;
  }
  if (t >= tf)
  {
    return 0;
  }
  double tau = t / tf;
  double qhat_dot = 30*pow(tau,4)-60*pow(tau,3)+30*pow(tau,2);
  double risultato=(qf - qi) * qhat_dot*1/tf;
  return risultato;

}