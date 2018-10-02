#include "headfile.h"

extern float SDS_OutData[4];
void SDS_OutPut_Data(float []);
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;//raw data
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

struct InfoStruct Info;//系统信息结构体
struct ParameterStruct Parameter;//系统参数结构体
struct FlagStruct Flag;//系统标志位结构体
struct FilterStruct Filter;//滤波参数结构体
//struct KFStruct KF;
extern void SystemInit();
extern void ShowMenu();
extern void ParameterSetting();
extern void MotorEnable();
 int main(void)
{
    
    get_clk();              //获取时钟频率 必须执行
    SystemInit();
    ParameterSetting();//参数初始化
    Flag.start=0;
    //Flag.FlagMotor=1;
    //Parameter.ExpectSpeed=220;
    for(;;){
      
        if(Flag.FlagMotor==-1)Parameter.ExpectSpeed=0;
        //MotorEnable();//电机使能 角度+-200 持续1500ms
        ShowMenu();//显示oled
        if(Flag.FlagSendData){//发送数据到示波器
       //8 10 11 
//        SDS_OutData[0]=adc_once(ADC0_SE15,ADC_12bit);//1
 //       SDS_OutData[1]=adc_once(ADC0_SE12,ADC_12bit);//F4 悬空
//        SDS_OutData[2]=adc_once(ADC0_SE13,ADC_12bit);//F5 悬空
//        SDS_OutData[3]=adc_once(ADC0_SE14,ADC_12bit);                   //f45    ADC0_SE12,   //F4 ADC0_SE13,   //F5
//        SDS_OutData[0]=adc_once(ADC0_SE8,ADC_12bit);//3
//        SDS_OutData[1]=adc_once(ADC0_SE10,ADC_12bit);//2
//        SDS_OutData[2]=adc_once(ADC0_SE11,ADC_12bit);//4
//        SDS_OutData[3]=adc_once(ADC0_SE9,ADC_12bit);  //5        
        //SDS_OutData[3]=mpu_gy*ro_x-Info.gyro_x_offset;
        
        
        SDS_OutPut_Data(SDS_OutData);
        SDS_OutData[0]=Info.AD[2] ;           
        SDS_OutData[1]= Info.AD[4];
        SDS_OutData[2]= Info.AD[1];
        SDS_OutData[3]= Info.AD[5];
        
        Flag.FlagSendData=0;//清除标志位
        //if(!gpio_get(D5))buzzer();
        }
    }
}
