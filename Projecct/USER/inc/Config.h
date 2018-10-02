#ifndef _CONFIG_H
#define _CONFIG_H
#include "headfile.h"
#define BAUD 9600
#define MotorDuty0 10000 //电机零速占空比
#define MotorFreq 15000  //电机频率
#define Acc_Z_Offset -857//-557                   //-527 //直立中值
#define dt 0.005 //积分时间
#define COE 41.7831325//40.6054688//50//209.421842//779.62963//10//259.803922//291.22449//10//261.960784//64.2592593//10//63.7735849//10//62.962963
#define K 0.008 //互补系数
#define UARTRecvDataLength 4
#define SpeedPeriod 12 //12 * 5ms
#define TurnPeriod 5  //5*1ms
#define GetSpeedPeriod 4
#define GetDataPeriod 10
#define MaxSpeedAngle 150//最大速度角
#define StartTimeMs 1500 
#define TurnOffset 0
#define SlidingFilterWidth 10
#define TurnDutyK 1
#define AD15MIN 300
extern struct ParameterStruct Parameter;
struct InfoStruct
{
   int16 acc_x;
   int16 acc_y;
   int16 acc_z;
   int16 angle_last;
   int16 angle;
   int16 gyro_x;
   int16 gyro_y;
   int16 gyro_z;
   int16 offset;
   int16 speedL;
   int16 speedR;
   int16 speed;
   int16 speedLast;
   int16 acc_z_offset;
   float gyro_y_offset;
   float gyro_z_offset;
   float gyro_x_offset;
   int32 balancepoint;
   int32 dutyL;
   int32 dutyR;
   int32 balanceduty;
   int32 speedduty;
   int32 turnduty;
   int32 turndutyLast;
   int16 delta;
   float speedangle;
   float speedangleLast;
   float angleIntegral;
   uint8 uartbuff[8];
   uint8 dip[5];
   uint32 timer;
   int16 AD[6];
   int16 buff[6][SlidingFilterWidth];
   int SpeedIntegral;
   int P_Speed_Output;
   int16 SpeedErrorBuff[15];
   int32 SubTotal;
   int32 DirIntegral;
   int16 offsetmiddle;
   int16 offsetside;
   int16 ExpectSpeed;
   int16 ExpectAD;
};
struct ParameterStruct{
    int16 P_Balance;
    int16 D_Balance;
    int16 P_Speed;
    int16 I_Speed;
    int16 P_Turn;
    int16 D_Turn;
    int16 ExpectSpeed;
    int16 ExpectOffset;
};
struct FlagStruct{
    vuint8 FlagMs;
    vuint8 FlagSendData;
    vuint8 FlagRecvData;
    vuint8 FlagRecvDataFinish;
    vuint8 FlagSpeed;
    vuint8 FlagTurn;
    vint8 FlagMotor;
    vuint8 FlagGetData;
    vuint8 FlagGetSpeed;
    vuint8 FlagGetOffset;
    vint8 TurnEnable;
    vuint8 DirOutputLimit;
    vuint8 SP;
    vint16 Circle;
    vuint32 delay;
    vuint8 Integral;
    int32 time;
    vuint8 Bumpy;
    vint8 dir;
    vuint8 start;
    vuint8 count;
};
struct KFStruct{
    float Angle;
    float Gyro_x;
};

struct FilterStruct{
    int16 angleBuff[5];
};
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
//#define LIMIT((x),(min),(max)) {if((x)<(min))(x)=(min);if((x)>(max))(x)=(max);}
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define buzzeron() gpio_set(C6,1)
#define buzzeroff() gpio_set(C6,0)
#define buzzer() gpio_turn(C6)
#endif