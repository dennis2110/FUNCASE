#ifndef TRACK_LINE_FUZZY_H
#define TRACK_LINE_FUZZY_H

#include <iostream>

//#define DEBUG_VS15
#define DEBUG_ROS


#define NB_err_ang -0.3f
#define NS_err_ang -0.1f
#define ZO_err_ang 0.0f
#define PS_err_ang 0.1f
#define PB_err_ang 0.3f

#define NB_err_ran -0.10f
#define NS_err_ran -0.08f
#define ZO_err_ran 0.0f
#define PS_err_ran 0.08f
#define PB_err_ran 0.10f

#define NB_u -30
#define NS_u -10
#define ZO_u 0
#define PS_u 10
#define PB_u 30

using namespace std;

int linerulebaseU[] = { NB_u ,NB_u ,NB_u ,NS_u ,ZO_u ,
                    NB_u ,NS_u ,NS_u ,ZO_u ,PB_u ,
                    NB_u ,NS_u ,ZO_u ,PS_u ,PB_u ,
                    NS_u ,ZO_u ,PS_u ,PS_u ,PB_u ,
                    ZO_u ,PS_u ,PB_u ,PB_u ,PB_u };

class LineFuzzyCountrol {
public:
        LineFuzzyCountrol() : _errback(0.0) {
          LineFuzzyCountrol::rulebase();
        }
        ~LineFuzzyCountrol() {

        }
    void nowstatus(uint8_t sensor1,uint8_t sensor2,uint8_t sensor3,uint8_t sensor4,uint8_t sensor5,uint8_t sensor6);
    void fuzzify();
    float defuzzify();
private:
    void rulebase();


public:
    float fuzzyError_ang[5];
    float fuzzyError_ran[5];

private:
    int _fuzzyU[25];
    float _err;
    float _errdot;
    float _errback;
};

void LineFuzzyCountrol::nowstatus(uint8_t sensor1,uint8_t sensor2,uint8_t sensor3,uint8_t sensor4,uint8_t sensor5,uint8_t sensor6){
  _err = static_cast<float>(sensor1) + static_cast<float>(sensor2) + static_cast<float>(sensor3)
          - static_cast<float>(sensor4) - static_cast<float>(sensor5) - static_cast<float>(sensor6);//YAYAYAYAYAYAYAYA
  _errdot = _err - _errback;
  _errback = _err;
#ifdef DEBUG_ROS
  //printf("error_range: %4.3f  error_angle: %4.3f",static_cast<double>(_err_ran),static_cast<double>(_err_ang));
  //printf("\n");
#endif
  }

void LineFuzzyCountrol::fuzzify()
{
  for (int i = 0; i < 5; i++)
  {
    fuzzyError_ang[i] = 0;
    fuzzyError_ran[i] = 0;
  }
  //error_ang
  if (_err < NB_err_ang)
  {
    fuzzyError_ang[0] = 1;
  }
  if (_err > NB_err_ang && _err < NS_err_ang)
  {
    fuzzyError_ang[0] = (NS_err_ang - _err) / (NS_err_ang - NB_err_ang);
    fuzzyError_ang[1] = (_err - NB_err_ang) / (NS_err_ang - NB_err_ang);
  }
  if (_err > NS_err_ang && _err < ZO_err_ang)
  {
    fuzzyError_ang[1] = (ZO_err_ang - _err) / (ZO_err_ang - NS_err_ang);
    fuzzyError_ang[2] = (_err - NS_err_ang) / (ZO_err_ang - NS_err_ang);
  }
  if (_err > ZO_err_ang && _err < PS_err_ang)
  {
    fuzzyError_ang[2] = (PS_err_ang - _err) / (PS_err_ang - ZO_err_ang);
    fuzzyError_ang[3] = (_err - ZO_err_ang) / (PS_err_ang - ZO_err_ang);
  }
  if (_err > PS_err_ang && _err < PB_err_ang)
  {
    fuzzyError_ang[3] = (PB_err_ang - _err) / (PB_err_ang - PS_err_ang);
    fuzzyError_ang[4] = (_err - PS_err_ang) / (PB_err_ang - PS_err_ang);
  }
  if (_err > PB_err_ang)
  {
    fuzzyError_ang[4] = 1;
  }
  //errordot_ran
  if (_errdot < NB_err_ran)
  {
    fuzzyError_ran[0] = 1;
  }
  if (_errdot > NB_err_ran && _errdot < NS_err_ran)
  {
    fuzzyError_ran[0] = (NS_err_ran - _errdot) / (NS_err_ran - NB_err_ran);
    fuzzyError_ran[1] = (_errdot - NB_err_ran) / (NS_err_ran - NB_err_ran);
  }
  if (_errdot > NS_err_ran && _errdot < ZO_err_ran)
  {
    fuzzyError_ran[1] = (ZO_err_ran - _errdot) / (ZO_err_ran - NS_err_ran);
    fuzzyError_ran[2] = (_errdot - NS_err_ran) / (ZO_err_ran - NS_err_ran);
  }
  if (_errdot > ZO_err_ran && _errdot < PS_err_ran)
  {
    fuzzyError_ran[2] = (PS_err_ran - _errdot) / (PS_err_ran - ZO_err_ran);
    fuzzyError_ran[3] = (_errdot - ZO_err_ran) / (PS_err_ran - ZO_err_ran);
  }
  if (_errdot > PS_err_ran && _errdot < PB_err_ran)
  {
    fuzzyError_ran[3] = (PB_err_ran - _errdot) / (PB_err_ran - PS_err_ran);
    fuzzyError_ran[4] = (_errdot - PS_err_ran) / (PB_err_ran - PS_err_ran);
  }
  if (_errdot > PB_err_ran)
  {
    fuzzyError_ran[4] = 1;
  }
#ifdef DEBUG_ROS
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ang%d=%f\t", i, static_cast<double>(fuzzyError_ang[i]));
  }
  printf("\n");
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ran%d=%f\t", i, static_cast<double>(fuzzyError_ran[i]));
  }
  printf("\n");
#endif

}

float LineFuzzyCountrol::defuzzify()
{
  float temp,sum=0;
  for (int edot = 0; edot < 5; edot++)
  {
    for (int e = 0; e < 5; e++)
    {
      temp = fuzzyError_ang[e] * fuzzyError_ran[edot] * static_cast<float>(_fuzzyU[edot * 5 + e]);
      sum += temp;
    }
  }
#ifdef DEBUG_ROS
  printf("sum: %4.3f",static_cast<double>(sum));
  printf("\n");
#endif
  return sum;
}

void LineFuzzyCountrol::rulebase()
{
  for (int i = 0; i < 25; i++)
  {
    _fuzzyU[i] = linerulebaseU[i];
  }
}
#endif // TRACK_LINE_FUZZY_H
