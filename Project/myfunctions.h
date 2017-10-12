#include "i2c_ee.h"
#include "math.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_it.h"


#include "time.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define HMC5883L_Addr	0x3C
#define L3G4200_Addr	0xD2
#define BMP085_Addr	0xEE
#define ADXL345_Addr	0xA6

#define gyro_CTRL_REG1 0x20
#define gyro_CTRL_REG2 0x21
#define gyro_CTRL_REG3 0x22
#define gyro_CTRL_REG4 0x23
#define gyro_CTRL_REG5 0x24

#define Gyro_Alpha 0.3
#define Accel_Alpha 0.5

#define ADXL345_DEVICE 0x53
#define ADXL345_TO_READ 6
 
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_DATAX0 0x32
#define ADXL345_DATA_FORMAT 0x31
 
#define ADXL345_OFSX 0x1E
#define ADXL345_OFSY 0x1F
#define ADXL345_OFSZ 0x20

#define PITCH 1
#define ROLL 2
#define YAW 3


#define PI 3.14159265

GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;
I2C_InitTypeDef I2C_InitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ErrorStatus HSEStartUpStatus;


typedef struct{
  double Pitch;
  double Roll;
  double Yaw;
} Axis_Values;  

typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
} Raw_G;  


//Gyro
int16_t Cur_RawGX, Cur_RawGY, Cur_RawGZ ;
double fXg,fYg,fZg;
double xg=0, yg=0, zg=0;
double Pre_AngleX = 0 , Pre_AngleY = 0, Pre_AngleZ = 0;
double Offset_GX = 0, Offset_GY=0, Offset_GZ=0;
int counter;

//Accel
int16_t Cur_RawAX, Cur_RawAY, Cur_RawAZ ; 
double fXa,fYa,fZa;
double xa=0, ya=0, za=0;
double _xOff = -0.023, _yOff =0 , _zOff = 0.03577027;

char myHex[10] = {'0','1','2','3','4','5','6','7','8','9'};

//Transciever
int MaxDuty =  210;
int MinDuty = 180;

//PWM Limits
int MaxDutyP = 270;
int MinDutyP = 120;

bool MotorStarter[4] = {FALSE,FALSE,FALSE,FALSE};
int baseValue =27;
int motorDuty[4] = {-2,-2,-2,-2};
//int motorDuty2 = -2;
//int motorDuty3 = -2;
//int motorDuty4 = -2;
int SampleTime =1000;


//Delay Operations
void LongDelay(u32 nCount);
void Delayms(u32 m);

//Initialization of pins
void Init_Pin(uint16_t Pin_Num,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed,
              GPIO_TypeDef* Port, uint32_t Port_Clock);

//Timers

void Init_Timer(int Freq_kHz, int Duty_Cycle_Percent,TIM_TypeDef * Timer,uint32_t Timer_Clock, int Channel);
void Init_Timer_Prescaler(double Freq_Hz,double Period, TIM_TypeDef * Timer,uint32_t Timer_Clock);
void Timer_Disable(TIM_TypeDef* Timer);
void Init_Timer_MS(TIM_TypeDef * Master, TIM_TypeDef * Slave, 
                   int Period,int Pulse, uint32_t Slave_Clock, uint16_t Trigger_Select);
void Interrupt_Attach(IRQn_Type IRQn, int Priority);
void Duty_Change(TIM_TypeDef* Timer, uint16_t Channel, int Freq_kHz, int Duty_Cycle_Percent);

//External Channel
void Set_EXT_Channel(uint32_t Ext_Channel,uint8_t Port_Source, uint8_t Pin_Source,EXTITrigger_TypeDef Trigger);

//RCC Config (From lab6. Possibly required for the GY-80 so call it in main
void RCC_Configuration(void);

//Gyroscope
void Gyro_Init(int scale);
void Gyro_Init_Test(double xoffset,double yoffset,double zoffset);
Axis_Values Get_Gyro_Val();

void ADXL_Init(char x_offset, char y_offset, char z_offset );
Axis_Values Get_Accel_Val();

//LCD Print
void LCD_PrintP3(int type, double number, int xLevel);
void LCD_PrintN3(int type, double number, int xLevel);
void LCD_PrintN3();

//Transmitter
void transmit_pwm();

//PID
double Setpoint = 0; 
double roll_PID, pitch_PID; //input
double ROLL_Output, PITCH_Output; //error values delivered to output
double ROLL_ITerm, PITCH_ITerm;
double outMin=-250, outMax=250; 
double ROLL_LastInput, PITCH_LastInput; // derivative of error is equal to the negative derivative of input except when setpoint is changing (avoids sending out a huge output spike)
double ROLL_Output_angle, PITCH_Output_angle;
double kp, ki, kd;

void Compute(); //Calculate angle error since last computation 
void PID_pwm();
void SetTunings(double Kp, double Ki, double Kd);//Set Kp, Ki, Kd (want P constant to be highest)
void SetSampleTime(); //save computation time by avoiding to multiply and divide by timechange at each calculation


/*Dump Code*/
//      if(pitchG<0)
//          LCD_PrintN3(1, pitchG, 0); 
//      else
//           LCD_PrintP3(1, pitchG, 0);       
//      
//       if(rollG<0)
//          LCD_PrintN3(2, rollG, 2);      
//       else
//          LCD_PrintP3(2, rollG, 2);
//      
//       if(yawG<0)
//          LCD_PrintN3(3, yawG, 4);    
//       else
//          LCD_PrintP3(3, yawG, 4);   
//       
//      if(rollA<0)
//          LCD_PrintN3(1, rollA, 6); 
//      else
//           LCD_PrintP3(1, rollA, 6);        


//       LCD_Clear();
//       LCD_PrintP3(3, baseValue, 4); 