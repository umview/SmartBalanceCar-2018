#include "SensorOperation.h"
extern struct InfoStruct Info;
extern struct ParameterStruct Parameter;
extern struct FilterStruct Filter;
extern struct FlagStruct Flag;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;
/***********************************/
int16 SlidingFilter(int16 buff[],uint8 n,int16 data){
  uint8 i=0;
  uint32 sum=0;
  for(i=0;i<n-1;i++)buff[i]=buff[i+1];
  buff[n-1]=data;
  for(i=0;i<n;i++)sum+=buff[i];
  return sum/n;
}
int16 DirConverge(int16 from,int16 to,float rate){
  if(rate>=1.0f)rate=1.0f;
  return (int16)(from*(1.0f-rate)+to*rate);
}
#define N 80
uint8 buff[40]={1,2,3,4,6,7,8,9,10,11,12,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13};
int16 GetOffset(){
  //Info.offset=(int16)（Info.AD[0]-Info.AD[1])/((float)(Info.AD[0]+Info.AD[1]));
  float submiddle,addmiddle,subside,addside;
 // static uint8 state=0;
 // static uint8 dir=0;
  int16 offset=0,offsetmiddle=0,offsetside=0;
  float rate=0.0f;
  uint32 sum[5]={0};

  uint8 i=0;
  
  for(i=0;i<5;i++){
    sum[0]+=adc_once(ADC0_SE15,ADC_12bit);  //F7  left     
    sum[1]+=adc_once(ADC0_SE10,ADC_12bit);  //C2  
    sum[2]+=adc_once(ADC0_SE9,ADC_12bit);   //C1
    sum[3]+=adc_once(ADC0_SE8,ADC_12bit);   //C0
    sum[4]+=adc_once(ADC0_SE14,ADC_12bit);  //F6 right
  }
  
  Info.AD[2]=SlidingFilter(Info.buff[0],SlidingFilterWidth,sum[0]/5);  //left
  Info.AD[1]=SlidingFilter(Info.buff[1],SlidingFilterWidth,sum[1]/5);
  Info.AD[3]=SlidingFilter(Info.buff[2],SlidingFilterWidth,sum[2]/5);
  Info.AD[5]=SlidingFilter(Info.buff[3],SlidingFilterWidth,sum[3]/5);
  Info.AD[4]=SlidingFilter(Info.buff[4],SlidingFilterWidth,sum[4]/5);  //right

  // Info.AD[2]=SlidingFilter(Info.buff[1],SlidingFilterWidth,sum[1]/5);
 // Info.AD[4]=SlidingFilter(Info.buff[0],SlidingFilterWidth,sum[0]/5);

  //Info.AD[2]=sum[0]/5;
  //Info.AD[1]=sum[1]/5;
  //Info.AD[3]=sum[2]/5;
  //Info.AD[5]=sum[3]/5;
  //Info.AD[4]=sum[4]/5;
    //Info.AD[2]=Info.AD[3];
 for(i=1;i<6;i++){
   if(Info.AD[i]<10)Info.AD[i]=10;
  }
 //Info.AD[1]=Info.AD[1]<AD15MIN?AD15MIN:Info.AD[1];
 //Info.AD[5]=Info.AD[5]<AD15MIN?AD15MIN:Info.AD[5];
   uint32 sumad=0;
  sumad=Info.AD[1]+Info.AD[5];
 if(sumad<=25)Flag.FlagMotor=-1;
  /*****get offset***********/
  subside=Info.AD[5]-Info.AD[1];
  addside=Info.AD[5]+Info.AD[1];//+Info.AD[3];
  submiddle=Info.AD[2]-Info.AD[4];
  addmiddle=Info.AD[4]+Info.AD[2];//+Info.AD[3];
  offsetside=-((int16)(subside*333.0f/addside)-TurnOffset);
  offsetmiddle=((int16)(submiddle*333.0f/addmiddle)-TurnOffset);
  Info.offsetmiddle=offsetmiddle;
  Info.offsetside=offsetside;
  offset=offsetmiddle;
  offset=offsetside;
  //if(Info.dip[4])offset=(int16)(offsetmiddle);
  if(Info.AD[1]==10&&Info.AD[5]==10&& Info.AD[2]==10&&Info.AD[4]==10)
    Flag.FlagMotor=-1;
  /*************************************/
  /*
  if(Info.AD[1]+Info.AD[5]>4600&& Flag.Circle==0)
  {
    Flag.Circle=1;
    buzzer();
  }
  if( Flag.Circle==1){
   Flag.delay++; 
   offsetmiddle=LIMIT(offsetmiddle,-180,180);
   offset=(int16)(offsetmiddle);
  }
  if(Flag.delay>70){
    Flag.delay=0;
    Flag.Circle=0;
    buzzer();
  }
    */
  static int32 subtotal[2]={0};
  int16 sub=0;
  
  if(((ABS(subtotal[1])<ABS(subtotal[0]))&&Flag.Circle==-1&&Flag.time==0)){
    Flag.Circle=1;                   //到达圆环中点
    //Flag.Integral=1;
    if(Info.SubTotal<0)Flag.dir=1;
    else if(Info.SubTotal>0)Flag.dir=-1;
    else Flag.dir=0;
  }
  if(Flag.Integral==1){
    Info.DirIntegral+= (int32)((mpu_gyro_x-Info.gyro_x_offset)/50.0f);
 }
 if( ABS(Info.DirIntegral)>1900){
   Info.DirIntegral=0;
   Flag.Integral=0;
   Flag.time=1;
 }
 if(Flag.time>=1){
   Flag.Circle=0;
    Flag.time++;
  if(Flag.time>500)Flag.time=0;
 }
  if((Info.AD[1]+Info.AD[5]>Info.ExpectAD) && Flag.Circle==0 && !Flag.delay){ 
    Flag.Circle=-1;
  }
  if(Flag.Circle==-1){
    sub=Info.AD[1]-Info.AD[5];
    Info.SubTotal+=sub;
    //Flag.Circle++;
    //if(ABS(sub)<20)Flag.Circle=-1;
  }else{
    //Flag.Circle=0;
    Info.SubTotal=0;
    subtotal[0]=0;
    subtotal[1]=0;
  }
  subtotal[0]=subtotal[1];
  subtotal[1]=Info.SubTotal;
  if(Flag.Circle==1&&Flag.delay==0){
    subtotal[0]=0;
    subtotal[1]=0;
    Info.SubTotal=0;
    Flag.Circle=0;
   // Flag.dir=0;
    if(Info.dip[4])Flag.delay++;
    buzzer();
  }

  if(Flag.delay){
    //if(Flag.delay<40)offset
    Flag.delay++;
    if(Flag.delay<60){///9
    if(Flag.dir==1){
      offsetmiddle=offsetmiddle>0?(int16)(offsetmiddle*25):offsetmiddle;
    }else if(Flag.dir==-1){
      offsetmiddle=offsetmiddle<0?(int16)(offsetmiddle*25):offsetmiddle;
    }
    offsetmiddle=LIMIT(offsetmiddle,-110,110);
    offset=(int16)(offsetmiddle);
    //if(Flag.delay>5)offset=offsetmiddle;
    }else{
      Info.gyro_y/=25;
    }
  }
  if(Flag.delay>70){
    Flag.delay=0;
    Flag.dir=0;
    buzzer();
  }
  
 /* 
if(Info.AD[3]>1600&&Flag.Circle==0&&Flag.delay==0)
{
          Flag.Circle++;
          //Flag.delay++;
          buzzer();
        
}
if(Flag.Circle){
  //offset=(int16)(offsetmiddle*1.0f);
  Flag.Circle++;
}
if(Flag.Circle==110){
  Flag.Circle=0;
  buzzer();
}

*/
  //offset=((int16)(submiddle/addmiddle*333)-TurnOffset)*5;
/*
if(Flag.Circle)
{
       Flag.Circle++;
       Flag.delay++; 
       if(Flag.Circle>30&&Flag.Circle<71){
        rate+=0.03333;
        offset=DirConverge(offsetside,offsetmiddle,rate);}
       if(Flag.Circle==70)rate=0;
       if(Flag.Circle>70)
        offset=offsetmiddle;
       if(Flag.Circle>140&&Flag.Circle<171){
         rate+=0.03333;
         offset=DirConverge(offsetmiddle,offsetside,rate);}
  // offset=(int16)(subside/addside*333)-TurnOffset;
  //if(Flag.Circle>49&&Flag.Circle<250)  
  //if(Flag.Circle>59&&Flag.Circle<100)
  //offset=(int16)(submiddle/addmiddle*333)-TurnOffset;
  //else offset=(int16)(submiddle/addmiddle*333)-TurnOffset; 
       if(Flag.Circle ==170){
          Flag.Circle=0;
          gpio_turn(G0);
          gpio_turn(G1);
          gpio_turn(G2);
          gpio_turn(G3);
          rate=0;}
 } 

if(Flag.delay>=170)Flag.delay++;

if(Flag.delay==1500)Flag.delay=0;
*/
  /************************BUMPY*************/
  if(ABS(Info.gyro_y)>300&&!Flag.Bumpy)Flag.Bumpy=1;
  if(Flag.Bumpy){
    Flag.Bumpy++;
   // buzzer();
    offset/=2.5f;
    Info.speedangle/=1.01f;
    if(Flag.Bumpy>50)Flag.Bumpy=0;
  }//else buzzeroff();
    return offset;
}       
void SpecialProcess(){
  static int32 SubTotal=0;
  if(Flag.SP)Flag.SP=0;
  else Flag.SP=100;
  if(Flag.Circle==0){
    SubTotal=0;
  }
  SubTotal+=(Info.AD[0]-Info.AD[1]);
}
/**********for kalman**********/
float    AngleFilter=0;      //融合后得到的角度
float  Gyro_x=0;

float  Q_angle=0.1; //1--0.001 过程噪声的协方差//0.2//0.001
                     //数值越小，变化越慢
float  Q_gyro =0.2;  //0.3--0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵,//0.1//0.05//0.4
                        //数值越小，变化越慢
float  R_angle=0.05;    //0.01--0.5   测量噪声的协方差 既测量偏差 //0.5//0.4
                        //数值越小，变化越快
//float  dt=0.005;       //0.001--0.005;为周期时间，对应5ms	                  
float  C_0 = 1.0;
float  Q_bias=0, Angle_err=0;
float  PCt_0=0.0, PCt_1=0, E=0.0;
float  K_0=0.0, K_1=0.0, t_0=0.0, t_1=0.0;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1.0, 0 },{ 0, 1.0 } };
void KalmanFilter(float Accel,float Gyro)		
{
    AngleFilter+=(Gyro - Q_bias) * dt;           //先验估计

    Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
    Pdot[1]=- PP[1][1];
    Pdot[2]=- PP[1][1];
    Pdot[3]=Q_gyro;
            
    PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
    PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
    PP[1][0] += Pdot[2] * dt;
    PP[1][1] += Pdot[3] * dt;
                    
    Angle_err =Accel-AngleFilter;     //zk-先验估计

    PCt_0 = C_0 * PP[0][0];
    PCt_1 = C_0 * PP[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * PP[0][1];
    
    PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
    PP[0][1] -= K_0 * t_1;
    PP[1][0] -= K_1 * t_0;
    PP[1][1] -= K_1 * t_1;
            
    AngleFilter	+= K_0 * Angle_err;	 //后验估计
    Q_bias	+= K_1 * Angle_err;	 //后验估计
    Gyro_x   = Gyro - Q_bias;	         //输出值(后验估计)的微分=角速度   
    Info.angle=(int16)(AngleFilter);
}

void ComplementaryFusionFilter(){//互补融合滤波
  static float gyro_y_last=0;
  Info.angle_last=Info.angle;
  Info.acc_z=Info.acc_z_offset-mpu_acc_z;//减去偏移得到实际角度
  gyro_y_last=Info.gyro_y;
  Info.gyro_y=(int16)(mpu_gyro_y-Info.gyro_y_offset);//减去偏移得到实际角速度
  Info.angleIntegral+=(mpu_gyro_y-Info.gyro_y_offset)*dt*COE;//对角速度积分得到参考角度 用于整定COE的值
  //KalmanFilter((float)(Info.acc_z),mpu_gyro_y-Info.gyro_y_offset);

  /********************/
  //angle+=(mpu_gyro_y-Info.gyro_y_offset)*dt*COE;
  Info.angle=(int16)((Info.angle+(Info.gyro_y*1.00f+gyro_y_last*0.0f)*dt*COE)*(1-K)+Info.acc_z*K);//互补融合
  Info.delta=(int16)(Info.angle-Info.speedangle);
  //Info.angle=(int16)(Info.angle*0.5+Info.angle_last*0.5);
  /*
  for(i=0;i<4;i++){
    Filter.angleBuff[i]=Filter.angleBuff[i+1];
  }
  Filter.angleBuff[4]=Info.angle;
  Info.angle=(Filter.angleBuff[0]+Filter.angleBuff[1]+Filter.angleBuff[2]+Filter.angleBuff[3]+Filter.angleBuff[4])/5;
  */
}
void RecvData(){//用于接收数据
  /*
  uart_getchar(uart0,Info.uartbuff+Flag.FlagRecvDataFinish);
  Flag.FlagRecvDataFinish++;
  if(Flag.FlagRecvDataFinish==UARTRecvDataLength)Flag.FlagRecvDataFinish=0;
  */
  static uint8 buff;
  //static uint8 lastbuff[2];
  //lastbuff[0]=Info.uartbuff[1];
  //lastbuff[1]=Info.uartbuff[2];
  Info.uartbuff[7]++;
  uart_getchar(uart0,&buff);
  if(buff==0x01 && Flag.FlagRecvData==0)Flag.FlagRecvData=1,Info.uartbuff[0]=0x01;//解析数据帧
  else if(Flag.FlagRecvData==1)Info.uartbuff[Flag.FlagRecvData++]=buff;
  else if(Flag.FlagRecvData==2)Info.uartbuff[Flag.FlagRecvData++]=buff;
  else if(Flag.FlagRecvData==3)Info.uartbuff[Flag.FlagRecvData++]=buff;
  if(Flag.FlagRecvData==4 && Info.uartbuff[3]==0xff)Flag.FlagRecvDataFinish=1;
  else Flag.FlagRecvDataFinish=0;
    //if(Info.uartbuff[1]==124 || Info.uartbuff[1]==125)Info.uartbuff[1]=lastbuff[0];
    //if(Info.uartbuff[2]==124 || Info.uartbuff[2]==125)Info.uartbuff[2]=lastbuff[1];
  //else Flag.FlagRecvDataFinish=0;
  if(Flag.FlagRecvData==4)Flag.FlagRecvData=0;
  //if(Flag.FlagRecvData==4)Flag.FlagRecvData==0;

  //if(Flag.FlagRecvData==4)Flag.FlagRecvData=0,Flag.FlagRecvDataFinish=1;
}
void GetSpeed(){//采集速度
        static int16 speedLast=0;
        speedLast=Info.speed;
        Info.speedL = ftm_count_get(ftm0);
        Info.speedR = ftm_count_get(ftm1);
        //计数器清零
        ftm_count_clean(ftm0);
        ftm_count_clean(ftm1);
        //根据方向信号判断正负，假设方向信号是高电平时为反转
        if(!gpio_get(C5))    Info.speedL = -((int16)(Info.speedL));//速度取负
        //else                speedl = temp1;             
        if(gpio_get(H5))    Info.speedR = -((int16)(Info.speedR));//速度取负
        //else                speedr = temp2;
        Info.speedLast=Info.speed;
        Info.speed=(Info.speedL+Info.speedR)/2;
        if(Flag.Bumpy){
          Info.speed=(int16)(0.01f*Info.speed+0.99f*speedLast);
          
        }
}

void SpeedOutputSmooth(){//系统占空比平滑输出
  static uint8 cnt=0;
  static float dValue;
  cnt++;
  if(cnt==1)dValue=(Info.speedangle-Info.speedangleLast)/(float)SpeedPeriod,Info.speedangle=Info.speedangleLast;
  Info.speedangle+=dValue;
  if(cnt==SpeedPeriod)cnt=0,Info.speedangleLast=Info.speedangle;
}
void DirectionOutputSmooth(){//系统占空比平滑输出
  static uint8 cnt=0;
  static float dValue;
  cnt++;
  if(cnt==1)dValue=(Info.turnduty-Info.turndutyLast)/(float)TurnPeriod,Info.turnduty=Info.turndutyLast;
  Info.turnduty+=dValue;
  if(cnt==TurnPeriod)cnt=0,Info.turndutyLast=Info.turnduty;
}
void MotorEnable(){
  static uint32 tag;
  if(Info.angle<200 && Info.angle>-200 && Flag.FlagMotor!=1){
    if((Info.timer-tag)>StartTimeMs)Flag.FlagMotor=1;
    else Flag.FlagMotor=-1;
  }else{
    tag=Info.timer;
  }
}
void ShowMenu(){
  static uint16 i=0;
  i++;
  if(i>65534){
 //   if(Info.dip[4]){
  OLED_P6x8Str(0,0,"PB");OLED_Print_Num1(12,0,Parameter.P_Balance);
  OLED_P6x8Str(54,0,"PD");OLED_Print_Num1(66,0,Parameter.D_Balance);
  OLED_P6x8Str(0,1,"PHZ");OLED_Print_Num1(18,1,Info.acc_z_offset);
  OLED_P6x8Str(60,1,"SA");OLED_Print_Num1(72,1,(int16)Info.speedangle);
  OLED_P6x8Str(0,2,"PS");OLED_Print_Num1(12,2,Parameter.P_Speed);
  OLED_P6x8Str(54,2,"ES");OLED_Print_Num1(66,2,Info.ExpectSpeed);
  OLED_P6x8Str(0,3,"PI");OLED_Print_Num1(12,3,Parameter.I_Speed);
  OLED_P6x8Str(54,3,"EO");OLED_Print_Num1(66,3,Parameter.ExpectOffset);
  OLED_P6x8Str(0,4,"PT");OLED_Print_Num1(12,4,Parameter.P_Turn);
  OLED_P6x8Str(54,4,"DT");OLED_Print_Num1(66,4,Parameter.D_Turn); 
  OLED_P6x8Str(0,5,"EAD");OLED_Print_Num1(30,5,Info.ExpectAD);
  OLED_Print_Num1(0,6,Info.AD[1]);OLED_Print_Num1(36,6,Info.AD[5]);
  OLED_Print_Num1(0,7,Info.AD[2]);OLED_Print_Num1(36,7,Info.AD[4]);  

  //OLED_P6x8Str(0,4,"PI");OLED_Print_Num1(12,4,Parameter.I_Speed);
  //OLED_P6x8Str(54,4,"EO");OLED_Print_Num1(66,4,Parameter.ExpectOffset);
  //OLED_P6x8Str(0,2,Info.uartbuff);
  //if(Flag.FlagRecvDataFinish==1){

  //}
  //OLED_Print_Num1(72,7,Info.uartbuff[7]);

  //OLED_P6x8Str(0,0,"PB");OLED_Print_Num1(12,0,Parameter.P_Balance);
  Info.dip[4]=gpio_get(D4);//循环更新拨码开关的值，用于减小中断时间
  Info.dip[3]=gpio_get(D3);
  Info.dip[2]=gpio_get(F1);
  Info.dip[1]=gpio_get(F0);
  i=0;
  }
}