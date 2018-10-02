#ifndef _SENSOROPERATION_H
#define _SENSOROPERATION_H
#include "headfile.h"

void ComplementaryFusionFilter();
void ShowMenu();
void KalmanFilter(float Accel,float Gyro);
void RecvData();
void GetSpeed();
void SpeedOutputSmooth();
void DirectionOutputSmooth();
void MotorEnable();
int16 GetOffset(void);
int16 SlidingFilter(int16 buff[],uint8 n,int16 data);
int16 DirConverge(int16 from,int16 to,float rate);
void SpecialProcess(void);
#endif