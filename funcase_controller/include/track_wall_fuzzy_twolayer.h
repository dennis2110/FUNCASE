#ifndef TRACK_WALL_FUZZY_TWOLAYER_H
#define TRACK_WALL_FUZZY_TWOLAYER_H

#include <iostream>

#define DEBUG_ROS

#define NB_dot_ang -0.1f
#define NS_dot_ang -0.03f
#define ZO_dot_ang 0.0f
#define PS_dot_ang 0.03f
#define PB_dot_ang 0.1f

#define NB_dot_ran -0.05f
#define NS_dot_ran -0.02f
#define ZO_dot_ran 0.0f
#define PS_dot_ran 0.02f
#define PB_dot_ran 0.05f

#define NB_op1 -10
#define NS_op1 -5
#define ZO_op1 0
#define PS_op1 5
#define PB_op1 10

/////////////////////////////////

#define NB_firstOP -20
#define NS_firstOP -10
#define ZO_firstOP 0
#define PS_firstOP 10
#define PB_firstOP 20

#define NB_err_ran -0.08f
#define NS_err_ran -0.05f
#define ZO_err_ran 0.0f
#define PS_err_ran 0.05f
#define PB_err_ran 0.08f

#define NB_op2 -40
#define NM_op2 -30
#define NS_op2 -20
#define ZO_op2 0
#define PS_op2 20
#define PM_op2 30
#define PB_op2 40

using namespace std;
///////////////////////
///           ang_dot
///          ---------
/// ran_dot |   rule  |
///          ---------
///////////////////////
///////////////////////
///           firstOP
///          ---------
/// err_ran |   rule  |
///          ---------
///////////////////////
///////////////////////
///            e
///        ---------
/// edot  |   rule  |
///        ---------
///////////////////////
int rulebaseOP1[] = { NB_op1 ,NB_op1 ,NB_op1 ,NS_op1 ,ZO_op1 ,
                      NB_op1 ,NS_op1 ,NS_op1 ,ZO_op1 ,PB_op1 ,
                      NB_op1 ,NS_op1 ,ZO_op1 ,PS_op1 ,PB_op1 ,
                      NB_op1 ,ZO_op1 ,PS_op1 ,PS_op1 ,PB_op1 ,
                      ZO_op1 ,PS_op1 ,PB_op1 ,PB_op1 ,PB_op1 };

int rulebaseOP2[] = { NB_op2 ,NB_op2 ,NS_op2 ,NS_op2 ,PS_op2 ,
                      NB_op2 ,NM_op2 ,NS_op2 ,PS_op2 ,PM_op2 ,
                      NB_op2 ,NS_op2 ,ZO_op2 ,PS_op2 ,PB_op2 ,
                      NM_op2 ,NS_op2 ,PS_op2 ,PM_op2 ,PB_op2 ,
                      NS_op2 ,PS_op2 ,PS_op2 ,PB_op2 ,PB_op2 };

class FuzzyCountrol {
public:
  FuzzyCountrol(){
    FuzzyCountrol::rulebase();
    _err_ang_bak = 0.0;
    _err_ran_bak = 0.0;
    _err_ang_dot = 0.0;
    _err_ran_dot = 0.0;
  }
  ~FuzzyCountrol(){

  }
  void nowstatus(float now_ang, float wall_ang, float now_ran, float wall_ran);
  void fuzzify();
  float defuzzify();
  void fuzzify1();
  float defuzzify1();
  void fuzzify2(float _op1);
  float defuzzify2();
private:
    void rulebase();

public:
    float fuzzyError_ang_dot[5];
    float fuzzyError_ran_dot[5];
    float fuzzyFirst_op1[5];
    float fuzzyError_ran[5];

private:
    int _fuzzyOP1[25];
    int _fuzzyOP2[25];
    float _err_ang;
    float _err_ran;
    float _err_ang_bak;
    float _err_ang_dot;
    float _err_ran_bak;
    float _err_ran_dot;
};

void FuzzyCountrol::nowstatus(float now_ang, float wall_ang, float now_ran, float wall_ran){
  _err_ang = wall_ang - now_ang;
  _err_ran = now_ran - wall_ran;

  _err_ang_dot = _err_ang - _err_ang_bak;
  _err_ang_bak = _err_ang;
  _err_ran_dot = _err_ran - _err_ran_bak;
  _err_ran_bak = _err_ran;
#ifdef DEBUG_ROS
  printf("error_range: %4.3f  error_range_dot: %4.3f  error_angle_dot: %4.3f",static_cast<double>(_err_ran),static_cast<double>(_err_ran_dot),static_cast<double>(_err_ang_dot));
  printf("\n");
#endif
  }

void FuzzyCountrol::fuzzify(){
  FuzzyCountrol::fuzzify1();
}

float FuzzyCountrol::defuzzify(){
  float op1 = FuzzyCountrol::defuzzify1();
  FuzzyCountrol::fuzzify2(op1);
  return FuzzyCountrol::defuzzify2();
}
void FuzzyCountrol::fuzzify1(){
  for (int i = 0; i < 5; i++)
  {
    fuzzyError_ang_dot[i] = 0;
    fuzzyError_ran_dot[i] = 0;
  }
  //error_ang_dot
  if (_err_ang_dot < NB_dot_ang)
  {
    fuzzyError_ang_dot[0] = 1;
  }
  if (_err_ang_dot >= NB_dot_ang && _err_ang_dot < NS_dot_ang)
  {
    fuzzyError_ang_dot[0] = (NS_dot_ang - _err_ang_dot) / (NS_dot_ang - NB_dot_ang);
    fuzzyError_ang_dot[1] = (_err_ang_dot - NB_dot_ang) / (NS_dot_ang - NB_dot_ang);
  }
  if (_err_ang_dot >= NS_dot_ang && _err_ang_dot < ZO_dot_ang)
  {
    fuzzyError_ang_dot[1] = (ZO_dot_ang - _err_ang_dot) / (ZO_dot_ang - NS_dot_ang);
    fuzzyError_ang_dot[2] = (_err_ang_dot - NS_dot_ang) / (ZO_dot_ang - NS_dot_ang);
  }
  if (_err_ang_dot >= ZO_dot_ang && _err_ang_dot < PS_dot_ang)
  {
    fuzzyError_ang_dot[2] = (PS_dot_ang - _err_ang_dot) / (PS_dot_ang - ZO_dot_ang);
    fuzzyError_ang_dot[3] = (_err_ang_dot - ZO_dot_ang) / (PS_dot_ang - ZO_dot_ang);
  }
  if (_err_ang_dot >= PS_dot_ang && _err_ang_dot < PB_dot_ang)
  {
    fuzzyError_ang_dot[3] = (PB_dot_ang - _err_ang_dot) / (PB_dot_ang - PS_dot_ang);
    fuzzyError_ang_dot[4] = (_err_ang_dot - PS_dot_ang) / (PB_dot_ang - PS_dot_ang);
  }
  if (_err_ang_dot >= PB_dot_ang)
  {
    fuzzyError_ang_dot[4] = 1;
  }

  //error_ran_dot
  if (_err_ran_dot < NB_dot_ran)
  {
    fuzzyError_ran_dot[0] = 1;
  }
  if (_err_ran_dot >= NB_dot_ran && _err_ran_dot < NS_dot_ran)
  {
    fuzzyError_ran_dot[0] = (NS_dot_ran - _err_ran_dot) / (NS_dot_ran - NB_dot_ran);
    fuzzyError_ran_dot[1] = (_err_ran_dot - NB_dot_ran) / (NS_dot_ran - NB_dot_ran);
  }
  if (_err_ran_dot >= NS_dot_ran && _err_ran_dot < ZO_dot_ran)
  {
    fuzzyError_ran_dot[1] = (ZO_dot_ran - _err_ran_dot) / (ZO_dot_ran - NS_dot_ran);
    fuzzyError_ran_dot[2] = (_err_ran_dot - NS_dot_ran) / (ZO_dot_ran - NS_dot_ran);
  }
  if (_err_ran_dot >= ZO_dot_ran && _err_ran_dot < PS_dot_ran)
  {
    fuzzyError_ran_dot[2] = (PS_dot_ran - _err_ran_dot) / (PS_dot_ran - ZO_dot_ran);
    fuzzyError_ran_dot[3] = (_err_ran_dot - ZO_dot_ran) / (PS_dot_ran - ZO_dot_ran);
  }
  if (_err_ran_dot >= PS_dot_ran && _err_ran_dot < PB_dot_ran)
  {
    fuzzyError_ran_dot[3] = (PB_dot_ran - _err_ran_dot) / (PB_dot_ran - PS_dot_ran);
    fuzzyError_ran_dot[4] = (_err_ran_dot - PS_dot_ran) / (PB_dot_ran - PS_dot_ran);
  }
  if (_err_ran_dot >= PB_dot_ran)
  {
    fuzzyError_ran_dot[4] = 1;
  }

#ifdef DEBUG_ROS
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ang_dot%d=%f\t", i, static_cast<double>(fuzzyError_ang_dot[i]));
  }
  printf("\n");
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ran_dot%d=%f\t", i, static_cast<double>(fuzzyError_ran_dot[i]));
  }
  printf("\n");
#endif
}

float FuzzyCountrol::defuzzify1(){
  float temp,sum=0;
  for (int edot = 0; edot < 5; edot++)
  {
    for (int e = 0; e < 5; e++)
    {
      temp = fuzzyError_ang_dot[e] * fuzzyError_ran_dot[edot] * static_cast<float>(_fuzzyOP1[edot * 5 + e]);
      sum += temp;
    }
  }
#ifdef DEBUG_ROS
  printf("sum1: %4.3f",static_cast<double>(sum));
  printf("\n");
#endif
  return sum;
}
void FuzzyCountrol::fuzzify2(float _op1){
  for (int i = 0; i < 5; i++)
  {
    fuzzyFirst_op1[i] = 0;
    fuzzyError_ran[i] = 0;
  }
  //first OP1
  if (_op1 < NB_firstOP)
  {
    fuzzyFirst_op1[0] = 1;
  }
  if (_op1 >= NB_firstOP && _op1 < NS_firstOP)
  {
    fuzzyFirst_op1[0] = (NS_firstOP - _op1) / (NS_firstOP - NB_firstOP);
    fuzzyFirst_op1[1] = (_op1 - NB_firstOP) / (NS_firstOP - NB_firstOP);
  }
  if (_op1 >= NS_firstOP && _op1 < ZO_firstOP)
  {
    fuzzyFirst_op1[1] = (ZO_firstOP - _op1) / (ZO_firstOP - NS_firstOP);
    fuzzyFirst_op1[2] = (_op1 - NS_firstOP) / (ZO_firstOP - NS_firstOP);
  }
  if (_op1 >= ZO_firstOP && _op1 < PS_firstOP)
  {
    fuzzyFirst_op1[2] = (PS_firstOP - _op1) / (PS_firstOP - ZO_firstOP);
    fuzzyFirst_op1[3] = (_op1 - ZO_firstOP) / (PS_firstOP - ZO_firstOP);
  }
  if (_op1 >= PS_firstOP && _op1 < PB_firstOP)
  {
    fuzzyFirst_op1[3] = (PB_firstOP - _op1) / (PB_firstOP - PS_firstOP);
    fuzzyFirst_op1[4] = (_op1 - PS_firstOP) / (PB_firstOP - PS_firstOP);
  }
  if (_op1 >= PB_firstOP)
  {
    fuzzyFirst_op1[4] = 1;
  }

  //error_ran
  if (_err_ran < NB_err_ran)
  {
    fuzzyError_ran[0] = 1;
  }
  if (_err_ran >= NB_err_ran && _err_ran < NS_err_ran)
  {
    fuzzyError_ran[0] = (NS_err_ran - _err_ran) / (NS_err_ran - NB_err_ran);
    fuzzyError_ran[1] = (_err_ran - NB_err_ran) / (NS_err_ran - NB_err_ran);
  }
  if (_err_ran >= NS_err_ran && _err_ran < ZO_err_ran)
  {
    fuzzyError_ran[1] = (ZO_err_ran - _err_ran) / (ZO_err_ran - NS_err_ran);
    fuzzyError_ran[2] = (_err_ran - NS_err_ran) / (ZO_err_ran - NS_err_ran);
  }
  if (_err_ran >= ZO_err_ran && _err_ran < PS_err_ran)
  {
    fuzzyError_ran[2] = (PS_err_ran - _err_ran) / (PS_err_ran - ZO_err_ran);
    fuzzyError_ran[3] = (_err_ran - ZO_err_ran) / (PS_err_ran - ZO_err_ran);
  }
  if (_err_ran >= PS_err_ran && _err_ran < PB_err_ran)
  {
    fuzzyError_ran[3] = (PB_err_ran - _err_ran) / (PB_err_ran - PS_err_ran);
    fuzzyError_ran[4] = (_err_ran - PS_err_ran) / (PB_err_ran - PS_err_ran);
  }
  if (_err_ran >= PB_err_ran)
  {
    fuzzyError_ran[4] = 1;
  }

#ifdef DEBUG_ROS
  for (int i = 0; i < 5; i++)
  {
    printf("Fst_OP1%d=%f\t", i, static_cast<double>(fuzzyFirst_op1[i]));
  }
  printf("\n");
  for (int i = 0; i < 5; i++)
  {
    printf("Err_ran%d=%f\t", i, static_cast<double>(fuzzyError_ran[i]));
  }
  printf("\n");
#endif
}

float FuzzyCountrol::defuzzify2(){
  float temp,sum=0;
  for (int edot = 0; edot < 5; edot++)
  {
    for (int e = 0; e < 5; e++)
    {
      temp = fuzzyFirst_op1[e] * fuzzyError_ran[edot] * static_cast<float>(_fuzzyOP2[edot * 5 + e]);
      sum += temp;
    }
  }
#ifdef DEBUG_ROS
  printf("sum2: %4.3f",static_cast<double>(sum));
  printf("\n");
#endif
  return sum;
}

void FuzzyCountrol::rulebase(){
  for (int i = 0; i < 25; i++)
  {
    _fuzzyOP1[i] = rulebaseOP1[i];
    _fuzzyOP2[i] = rulebaseOP2[i];
  }
}
#endif // TRACK_WALL_FUZZY_TWOLAYER_H
