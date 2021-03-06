#ifndef TRACK_WALL_FUZZY_TWOLAYER_H
#define TRACK_WALL_FUZZY_TWOLAYER_H
#include <iostream>

//#define DEBUG_VS15
#define DEBUG_ROS

#define NB_err_ran -0.10f
#define NS_err_ran -0.05f
#define ZO_err_ran 0.0f
#define PS_err_ran 0.05f
#define PB_err_ran 0.10f

#define NB_dot_ran -0.2f
#define NS_dot_ran -0.05f
#define ZO_dot_ran 0.0f
#define PS_dot_ran 0.05f
#define PB_dot_ran 0.2f



#define NB_u -40
#define NS_u -20
#define ZO_u 0
#define PS_u 20
#define PB_u 40

using namespace std;
///////////////////////
///            e
///        ---------
/// edot  |   rule  |
///        ---------
int rulebaseU[] = { NB_u ,NB_u ,NB_u ,NS_u ,ZO_u ,
                    NB_u ,NS_u ,NS_u ,ZO_u ,PB_u ,
                    NB_u ,NS_u ,ZO_u ,PS_u ,PB_u ,
                    NB_u ,ZO_u ,PS_u ,PS_u ,PB_u ,
                    ZO_u ,PS_u ,PB_u ,PB_u ,PB_u };

class FuzzyCountrol {
public:
        FuzzyCountrol() {
          FuzzyCountrol::rulebase();
          _err_ran_bak = 0.0;
          //_err_ang_bak = 0.0;
        }
        ~FuzzyCountrol() {

        }
    void nowstatus(float now_ang, float wall_ang, float now_ran, float wall_ran);
    void fuzzify();
    float defuzzify();
private:
    void rulebase();


public:
    float fuzzyError_ran_dot[5];
    float fuzzyError_ran[5];

private:
    int _fuzzyU[25];
    //float _err_ang;
    float _err_ran;
    //float _err_ang_bak;
    //float _err_ang_dot;
    float _err_ran_bak;
    float _err_ran_dot;
};
void FuzzyCountrol::nowstatus(float now_ang, float wall_ang, float now_ran, float wall_ran){
  //_err_ang = wall_ang - now_ang;
  _err_ran = now_ran - wall_ran;

  //_err_ang_dot = _err_ang - _err_ang_bak;
  //_err_ang_bak = _err_ang;
  _err_ran_dot = _err_ran - _err_ran_bak;
  _err_ran_bak = _err_ran;
#ifdef DEBUG_ROS
  printf("error_range: %4.3f  error_range_dot: %4.3f",static_cast<double>(_err_ran),static_cast<double>(_err_ran_dot));
  printf("\n");
#endif
  }

void FuzzyCountrol::fuzzify()
{
  for (int i = 0; i < 5; i++)
  {
    fuzzyError_ran_dot[i] = 0;
    fuzzyError_ran[i] = 0;
  }
  //error_ran
  if (_err_ran < NB_err_ran)
  {
    fuzzyError_ran[0] = 1;
  }
  if (_err_ran > NB_err_ran && _err_ran < NS_err_ran)
  {
    fuzzyError_ran[0] = (NS_err_ran - _err_ran) / (NS_err_ran - NB_err_ran);
    fuzzyError_ran[1] = (_err_ran - NB_err_ran) / (NS_err_ran - NB_err_ran);
  }
  if (_err_ran > NS_err_ran && _err_ran < ZO_err_ran)
  {
    fuzzyError_ran[1] = (ZO_err_ran - _err_ran) / (ZO_err_ran - NS_err_ran);
    fuzzyError_ran[2] = (_err_ran - NS_err_ran) / (ZO_err_ran - NS_err_ran);
  }
  if (_err_ran > ZO_err_ran && _err_ran < PS_err_ran)
  {
    fuzzyError_ran[2] = (PS_err_ran - _err_ran) / (PS_err_ran - ZO_err_ran);
    fuzzyError_ran[3] = (_err_ran - ZO_err_ran) / (PS_err_ran - ZO_err_ran);
  }
  if (_err_ran > PS_err_ran && _err_ran < PB_err_ran)
  {
    fuzzyError_ran[3] = (PB_err_ran - _err_ran) / (PB_err_ran - PS_err_ran);
    fuzzyError_ran[4] = (_err_ran - PS_err_ran) / (PB_err_ran - PS_err_ran);
  }
  if (_err_ran > PB_err_ran)
  {
    fuzzyError_ran[4] = 1;
  }

  //error_ran_dot
  if (_err_ran_dot < NB_dot_ran)
  {
    fuzzyError_ran_dot[0] = 1;
  }
  if (_err_ran_dot > NB_dot_ran && _err_ran_dot < NS_dot_ran)
  {
    fuzzyError_ran_dot[0] = (NS_dot_ran - _err_ran_dot) / (NS_dot_ran - NB_dot_ran);
    fuzzyError_ran_dot[1] = (_err_ran_dot - NB_dot_ran) / (NS_dot_ran - NB_dot_ran);
  }
  if (_err_ran_dot > NS_dot_ran && _err_ran_dot < ZO_dot_ran)
  {
    fuzzyError_ran_dot[1] = (ZO_dot_ran - _err_ran_dot) / (ZO_dot_ran - NS_dot_ran);
    fuzzyError_ran_dot[2] = (_err_ran_dot - NS_dot_ran) / (ZO_dot_ran - NS_dot_ran);
  }
  if (_err_ran_dot > ZO_dot_ran && _err_ran_dot < PS_dot_ran)
  {
    fuzzyError_ran_dot[2] = (PS_dot_ran - _err_ran_dot) / (PS_dot_ran - ZO_dot_ran);
    fuzzyError_ran_dot[3] = (_err_ran_dot - ZO_dot_ran) / (PS_dot_ran - ZO_dot_ran);
  }
  if (_err_ran_dot > PS_dot_ran && _err_ran_dot < PB_dot_ran)
  {
    fuzzyError_ran_dot[3] = (PB_dot_ran - _err_ran_dot) / (PB_dot_ran - PS_dot_ran);
    fuzzyError_ran_dot[4] = (_err_ran_dot - PS_dot_ran) / (PB_dot_ran - PS_dot_ran);
  }
  if (_err_ran_dot > PB_dot_ran)
  {
    fuzzyError_ran_dot[4] = 1;
  }

#ifdef DEBUG_ROS
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ran%d=%f\t", i, static_cast<double>(fuzzyError_ran[i]));
  }
  printf("\n");
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ran_dot%d=%f\t", i, static_cast<double>(fuzzyError_ran_dot[i]));
  }
  printf("\n");
#endif

}

float FuzzyCountrol::defuzzify()
{
  float temp,sum=0;
  for (int edot = 0; edot < 5; edot++)
  {
    for (int e = 0; e < 5; e++)
    {
      temp = fuzzyError_ran[e] * fuzzyError_ran_dot[edot] * static_cast<float>(_fuzzyU[edot * 5 + e]);
      sum += temp;
    }
  }
#ifdef DEBUG_ROS
  printf("sum: %4.3f",static_cast<double>(sum));
  printf("\n");
#endif
  return sum;
}

void FuzzyCountrol::rulebase()
{
  for (int i = 0; i < 25; i++)
  {
    _fuzzyU[i] = rulebaseU[i];
  }
}
#endif // TRACK_WALL_FUZZY_TWOLAYER_H
