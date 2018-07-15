#ifndef _FILTER_H_
#define _FILTER_H_
#include "include.h"

#define KALMAN_Q 0.02

#define KALMAN_R 7.0000



float KalmanFilter (const float  ResrcData,Kalman_Date* Kalman);
double KalmanFilter1(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter2(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
double KalmanFilter3(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
float filter_1st(const float ResrcData, Filter_1st_Str *Fil);
double filter_1st1(const double ResrcData,float m_filter);
double filter_1st2(const double ResrcData,float m_filter);
double filter_1st3(const double ResrcData,float m_filter);
double filter_1st4(const double ResrcData,float m_filter);
double filter_ave1(const double ResrcData_ave);

#endif
