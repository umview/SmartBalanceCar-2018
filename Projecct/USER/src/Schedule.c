#include "Schedule.h"
extern struct InfoStruct Info;
extern struct ParameterStruct Parameter;
extern struct FlagStruct Flag;
void PIT_CH0_IRQHandler(void)
{    

    Info.timer++;//1ms计时器提供时间基准
    PIT_FlAG_CLR(pit0);
    //gpio_turn(G2); 
    Flag.FlagMs++;//毫秒标志位
    if(Flag.FlagMs>5)Flag.FlagMs=0;
      Get_Gyro();//imu数据采集角速度
    if(Flag.FlagMs==1){
      Flag.FlagSpeed++;
      Flag.FlagGetSpeed++;
      if(Flag.FlagGetSpeed>=GetSpeedPeriod)GetSpeed(),Flag.FlagGetSpeed=0;
        if(Flag.FlagSpeed>SpeedPeriod){
          SpeedCtrl();
                             //speedperiod*5 ms进行速度控制
          Flag.FlagSpeed=0;
        }
      SpeedOutputSmooth();//速度控制输出平均累加到Speedperiod中
    }else if(Flag.FlagMs==2){
      Get_AccData();
    }else if(Flag.FlagMs==3){
      ComplementaryFusionFilter();//5ms
    }else if(Flag.FlagMs==4){
      //Flag.FlagGetOffset++;
      //if(Flag.FlagGetOffset==2)
      Info.offset=GetOffset();//,Flag.FlagGetOffset=0;//10ms
     Info.offset=Info.dip[3]?0:Info.offset;
    }else if(Flag.FlagMs==5){
      //if(Flag.TurnEnable==1)TurnCtrl();//转向环 5ms
      //else Info.turnduty=0;

      TurnCtrl();
      Flag.FlagSendData=1;
    }
    AngleCtrl();
    DirectionOutputSmooth();
     //Info.turnduty=0;
    Info.dutyL=-(Info.balanceduty+Info.turnduty);//三环输出融合
    Info.dutyR=-(Info.balanceduty-Info.turnduty);
 
    
    if(Flag.FlagMotor==1)UpdataDuty();//更新占空比
    else {
      Info.dutyL=0;Info.dutyR=0;
      UpdataDuty();
    }
    if(Flag.start){
      Flag.count++;
      if(Flag.count>=25){
        Flag.count=0;
        Flag.start++;
        if(Flag.start>=128)Flag.start=128;
      }
    }
}

/*
void PIT_CH0_IRQHandler(void)
{
    Info.timer++;//1ms计时器提供时间基准
    PIT_FlAG_CLR(pit0);
    gpio_turn(G2); 
    Flag.FlagMs++;//毫秒标志位
    if(Flag.FlagMs>5)Flag.FlagMs=0;
      Get_Gyro();//imu数据采集角速度
      BalanceCtrl();//直立环 5ms
      GetOffset();
      TurnCtrl();//转向环 5ms

    if(Flag.FlagMs==1){
      ComplementaryFusionFilter();


    }else if(Flag.FlagMs==2){
      gpio_turn(G1);
      Flag.FlagSpeed++;
      Flag.FlagGetSpeed++;
    if(Flag.FlagGetSpeed>=4)GetSpeed(),Flag.FlagGetSpeed=0;
      if(Flag.FlagSpeed>SpeedPeriod){
        SpeedCtrl();//speedperiod*5 ms进行速度控制
        Flag.FlagSpeed=0;
      }
      OutputSmooth();//速度控制输出平均累加到Speedperiod中
      gpio_turn(G1);
    }else if(Flag.FlagMs==3){
      gpio_turn(G2);
   //   gpio_turn(G2);
    }else if(Flag.FlagMs==4){
      gpio_turn(G3);//5ms
      Get_AccData();//加速度
      gpio_turn(G3);//5ms

    }else if(Flag.FlagMs==5){
      gpio_turn(G0);


      //Info.dutyL=-1000;
      //Info.dutyR=-1000;

      Flag.FlagSendData=1;//发送数据
      gpio_turn(G0);
    }
      Info.dutyL=Info.balanceduty+Info.turnduty;//三环输出融合
      Info.dutyR=Info.balanceduty-Info.turnduty;
      if(Flag.FlagMotor==1)UpdataDuty();//更新占空比
      else {
        Info.dutyL=0;Info.dutyR=0;
        UpdataDuty();
      }
}*/
/*
void PIT_CH0_IRQHandler(void)
{
    Info.timer++;//1ms计时器提供时间基准
    PIT_FlAG_CLR(pit0);
    gpio_turn(G2); 
    Flag.FlagMs++;//毫秒标志位
    if(Flag.FlagMs>5)Flag.FlagMs=0;
    
    if(Flag.FlagMs==1){


    }else if(Flag.FlagMs==2){
      gpio_turn(G1);
      Flag.FlagSpeed++;
      Flag.FlagGetSpeed++;
    if(Flag.FlagGetSpeed>=4)GetSpeed(),Flag.FlagGetSpeed=0;
      if(Flag.FlagSpeed>SpeedPeriod){
        SpeedCtrl();//speedperiod*5 ms进行速度控制
        Flag.FlagSpeed=0;
      }
      OutputSmooth();//速度控制输出平均累加到Speedperiod中
      gpio_turn(G1);
    }else if(Flag.FlagMs==3){
      gpio_turn(G2);
      GetOffset();
      gpio_turn(G2);
    }else if(Flag.FlagMs==4){
      gpio_turn(G3);//5ms
      Get_Gyro();//imu数据采集角速度
      Get_AccData();//加速度
      gpio_turn(G3);//5ms

    }else if(Flag.FlagMs==5){
      gpio_turn(G0);

      ComplementaryFusionFilter();
      BalanceCtrl();//直立环 5ms
      TurnCtrl();//转向环 5ms
      Info.dutyL=Info.balanceduty+Info.turnduty;//三环输出融合
      Info.dutyR=Info.balanceduty-Info.turnduty;
      //Info.dutyL=-1000;
      //Info.dutyR=-1000;
      if(Flag.FlagMotor==1)UpdataDuty();//更新占空比
      else {
        Info.dutyL=0;Info.dutyR=0;
        UpdataDuty();
      }
      Flag.FlagSendData=1;//发送数据
      gpio_turn(G0);
    }
}
*/
/*
      Flag.FlagGetData++;
      if(Flag.FlagGetData>GetDataPeriod){//接受更新远程数据
        if(Info.dip[3]){
          if( Flag.FlagRecvDataFinish==1){
            Parameter.ExpectSpeed=(int16)(1.3*(Info.uartbuff[1]-120));
            Parameter.ExpectOffset=Info.uartbuff[2]-120;
            Flag.FlagRecvDataFinish=0;
          }
        }else{
            Parameter.ExpectSpeed=0;
            Parameter.ExpectOffset=0;
        }
        Flag.FlagGetData=0;
      }
*/
void UART0_IRQHandler(){
  //Flag.FlagRecvData=1;
  RecvData();
}
void AngleCtrl(){

  Info.balanceduty=Parameter.P_Balance * (int32)(Info.delta) / 2 + (int32)(Parameter.D_Balance) * Info.gyro_y / 5;
  Info.balanceduty=LIMIT(Info.balanceduty,-32760,32760);//输出限幅
}
void BalanceCtrl(){//p*（angle-speedangle）+d*omega
  Info.balanceduty=Parameter.P_Balance * (int32)(Info.angle-Info.speedangle) / 2 + (int32)(Parameter.D_Balance) * Info.gyro_y / 5;
  Info.balanceduty=LIMIT(Info.balanceduty,-32760,32760);//输出限幅
  //if(Info.balanceduty>32760)Info.balanceduty=32760;
  //if(Info.balanceduty<-32760)Info.balanceduty=-32760;
}
void SpeedCtrl(){//p*speederror+I*speederrorintegral
  int16 SpeedError;
  static uint8 i=0;
  uint8 j=0;
  //if(Parameter.ExpectSpeed!=0 && Flag.FlagMotor==1 && Info.speed>Parameter.ExpectSpeed*2)Info.speed=Info.speedLast;
  SpeedError=Info.speed-Parameter.ExpectSpeed;
  //if(SpeedError>0)SpeedError*=3;//提高减速响应
  Info.SpeedErrorBuff[i++]=SpeedError;
  if(i==15)i=0;
  if(Parameter.ExpectSpeed){
    Info.SpeedIntegral=0;
    for(j=0;j<15;j++)
    Info.SpeedIntegral+=Info.SpeedErrorBuff[j];
  }
  else Info.SpeedIntegral=0;
  Info.speedangleLast=Info.speedangle;
  Info.SpeedIntegral=LIMIT(Info.SpeedIntegral,-MaxSpeedAngle,MaxSpeedAngle);
  Info.P_Speed_Output=Parameter.P_Speed*SpeedError;
  Info.speedduty=(int32)(Info.P_Speed_Output/10.f+Parameter.I_Speed/100.0f * Info.SpeedIntegral);
  Info.speedduty=LIMIT(Info.speedduty,-MaxSpeedAngle,MaxSpeedAngle);//角度输出限幅
  Info.speedangle=(float)(Info.speedduty);
}
void TurnCtrl(){//p*offset+d*omega
  //if(Info.dip[2])Info.offset=0;
  Info.turndutyLast=Info.turnduty;
  //if(ABS(Info.gyro_y)>50 && ABS(ABS(Info.angle)-ABS(Info.speedangle))>120)Flag.DirOutputLimit=1,Info.offset=0;
  //else Flag.DirOutputLimit=0;
  Info.turnduty=(int16)(Parameter.P_Turn * Info.offset + Parameter.D_Turn * (mpu_gyro_x-Info.gyro_x_offset));  //
  Info.turnduty=(int32)(Info.turnduty*TurnDutyK+Info.turndutyLast*(1-TurnDutyK));
  Info.turnduty=LIMIT(Info.turnduty,-32760,32760);
}

