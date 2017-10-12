#include "myfunctions.h"

void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void Delayms(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

void Init_Clock_All_Ports(){
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE |RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG, ENABLE);  
}

void Init_Pin(uint16_t Pin_Num,GPIOMode_TypeDef Mode,GPIOSpeed_TypeDef Speed,
              GPIO_TypeDef* Port, uint32_t Port_Clock){
  RCC_APB2PeriphClockCmd(Port_Clock, ENABLE);
  GPIO_InitStructure.GPIO_Pin = Pin_Num; 
  GPIO_InitStructure.GPIO_Mode = Mode; 
  GPIO_InitStructure.GPIO_Speed = Speed; 
  GPIO_Init(Port, &GPIO_InitStructure);
}

void Init_Timer(int Freq_kHz, int Duty_Cycle_Percent,TIM_TypeDef * Timer,uint32_t Timer_Clock, int Channel){
 
  RCC_APB2PeriphClockCmd(Timer_Clock |RCC_APB2Periph_AFIO ,ENABLE);
  
  int Period_Val = (72000000)/(Freq_kHz*1000)-1;
  TIM_TimeBaseStructure.TIM_Period = Period_Val;//500;//Period_Val;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;//40000; //0;
  TIM_TimeBaseStructure.TIM_ClockDivision =0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(Timer, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = (Duty_Cycle_Percent*Period_Val)/100+1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  switch(Channel){
  case 1:
 
    TIM_OC1Init(Timer, &TIM_OCInitStructure);
    break;
  case 2:
 
    TIM_OC2Init(Timer, &TIM_OCInitStructure);
    break;
      case 3:
 
    TIM_OC3Init(Timer, &TIM_OCInitStructure);
    break;
       case 4:
 
    TIM_OC4Init(Timer, &TIM_OCInitStructure);
    break;
  }
  if(Timer==TIM1 || Timer==TIM8){
      RCC_APB2PeriphClockCmd(Timer_Clock, ENABLE);
      TIM_CtrlPWMOutputs(Timer,ENABLE);
      TIM_Cmd(Timer, ENABLE);  
  }
  else{
    RCC_APB1PeriphClockCmd(Timer_Clock, ENABLE);
    TIM_Cmd(Timer, ENABLE);
  }
  
//    TIM_TimeBaseStructure.TIM_Period = 11999;//500;//Period_Val;
//  TIM_TimeBaseStructure.TIM_Prescaler = 0;//40000; //0;
//  TIM_TimeBaseStructure.TIM_ClockDivision =0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = 6000; //(Duty_Cycle_Percent*Period_Val)/100+1;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//  TIM_Cmd(TIM3, ENABLE);
}

//Formula : TIM_Period = PWM_Period * (Prescaler+1/Clock_Source)
void Init_Timer_Prescaler(double Freq_Hz,double Period, TIM_TypeDef * Timer,uint32_t Timer_Clock){
    double Prescaler = ((72000000)/(Freq_Hz*Period))-1;
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = Period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(Timer, &TIM_TimeBaseStructure);
    TIM_Cmd(Timer, ENABLE);
    
}

void Init_Timer_MS(TIM_TypeDef * Master, TIM_TypeDef * Slave, int Period,int Pulse,
                   uint32_t Slave_Clock, uint16_t Trigger_Select){
    TIM_SelectMasterSlaveMode(Master, TIM_MasterSlaveMode_Enable);
    TIM_SelectOutputTrigger(Master, TIM_TRGOSource_Update);
   
    TIM_TimeBaseStructure.TIM_Period = Period;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision =0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(Slave, &TIM_TimeBaseStructure);
  
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = Pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(Slave, &TIM_OCInitStructure);
    TIM_OC2Init(Slave, &TIM_OCInitStructure);
    TIM_OC3Init(Slave, &TIM_OCInitStructure);
    TIM_OC4Init(Slave, &TIM_OCInitStructure);
    RCC_APB1PeriphClockCmd(Slave_Clock, ENABLE);
    TIM_Cmd(Slave, ENABLE);
    TIM_SelectSlaveMode(Slave, TIM_SlaveMode_Gated);
    TIM_SelectInputTrigger(Slave, Trigger_Select);
}

void Duty_Change(TIM_TypeDef* Timer, uint16_t Channel, int Freq_kHz, int Duty_Cycle_Percent){
    int Period_Val = (72000000)/(Freq_kHz*1000)-1;
    switch(Channel){
    case 1:
      Timer->CCR1 = (Duty_Cycle_Percent*Period_Val)/100+1; 
      break;    
    }
}

void Timer_Disable(TIM_TypeDef* Timer){
  TIM_Cmd(Timer, DISABLE);
}

void RCC_Configuration(void)
{
    /* RCC system reset(for debug purpose) */
//  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  { 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
#ifndef STM32F10X_CL  
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

#else
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 7 = 56 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_7);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}

void Interrupt_Attach(IRQn_Type IRQn , int Priority){
  
  NVIC_InitStructure.NVIC_IRQChannel = IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Priority;//Highest priority - depends on you application
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  return;
}

void Set_EXT_Channel(uint32_t Ext_Channel,uint8_t Port_Source, uint8_t Pin_Source,EXTITrigger_TypeDef Trigger  ){
  GPIO_EXTILineConfig(Port_Source, Pin_Source); 
  EXTI_InitStructure.EXTI_Line = Ext_Channel; 
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
  EXTI_InitStructure.EXTI_Trigger = Trigger; 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE; 
  EXTI_Init(&EXTI_InitStructure);
}



//Gyroscope Initialization
void Gyro_Init(int scale){
  I2C_ByteWrite(L3G4200_Addr, 0x0F, 0xD3);
  I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG1, 0x0F);
  switch(scale){
    case 250:
      I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG4, 0x80); break;
    case 500:
      I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG4, 0x10); break;
    case 2000:
      I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG4, 0x30); break;      
  }
  I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG5, 0x00);
  Delayms(10); 
  return;
}

void Gyro_Init_Test(double xoffset,double yoffset,double zoffset){

  Offset_GX=  xoffset;
  Offset_GY=yoffset;
  Offset_GZ=zoffset;
  
  I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG1, 0x0F);
  I2C_ByteWrite(L3G4200_Addr, gyro_CTRL_REG4, 0x80);
}


Axis_Values Get_Gyro_Val(){  
  //0.00875 Considering 250dps
 //time_Now = (double) clock()/CLOCKS_PER_SEC; 
  
 double pitch, roll, yaw;
 
 Cur_RawGX=((I2C_ByteRead(L3G4200_Addr,0x29)<<8) | 
               (I2C_ByteRead(L3G4200_Addr, 0x28)));
 Cur_RawGY=((I2C_ByteRead(L3G4200_Addr,0x2B)<<8) | 
               (I2C_ByteRead(L3G4200_Addr, 0x2A)));
 Cur_RawGZ=I2C_ByteRead(L3G4200_Addr,0x2D)<<8 | (I2C_ByteRead(L3G4200_Addr, 0x2C));
 
 
//Cur_RawGX = 1;
//Cur_RawGY = 1;
//Cur_RawGZ = 1;

  fXg = Cur_RawGX*0.00875 ;
  fYg = Cur_RawGY*0.00875 ;
  fZg =Cur_RawGZ*0.00875 ; 

  
  fXg = Gyro_Alpha*fXg + (1-Gyro_Alpha)*xg;
  xg = fXg; 
  fYg = Gyro_Alpha*fYg + (1-Gyro_Alpha)*yg;
  yg = fYg;  
  fZg = Gyro_Alpha*fZg + (1-Gyro_Alpha)*zg;
  zg = fZg;  
      
//  counter = TIM_GetCounter(TIM2);
//  
//   pitch = Pre_AngleX + fXg*(TIM_GetCounter(TIM2)*0.00001);
//   Pre_AngleX = pitch;
//   roll = Pre_AngleY + fYg*(TIM_GetCounter(TIM2)*0.00001);
//   Pre_AngleY = roll;
//   yaw = Pre_AngleZ + fZg*(TIM_GetCounter(TIM2)*0.00001);
//   Pre_AngleZ = yaw;    

  // TIM_Cmd(TIM8, DISABLE);
//   TIM_Cmd(TIM8, DISABLE);
//   TIM_SetCounter(TIM8,0);
//   TIM_SetCounter(TIM2,0);
//   TIM_Cmd(TIM8, ENABLE);
   
//   Init_Timer_MS(TIM4,TIM2,999,500,RCC_APB1Periph_TIM2,TIM_TS_ITR3);
  
   //Set the Inputs for PID
   
   //Return
   Axis_Values DPS;
   DPS.Pitch = fXg; 
   DPS.Roll = fYg;
   DPS.Yaw = fZg;
   
   return DPS;
}


void LCD_PrintP3(int type, double number, int xLevel){
  char myStringP[6] = {'P','i','t','c','h',':'};
  char myStringR[5] = {'R','o','l','l',':'};  
  char myStringY[4] = {'Y','a','w',':'}; 
  switch(type){
  case 1: 
        LCD_DrawString(xLevel,10,myStringP,6);
        break;
  case 2:
        LCD_DrawString(xLevel,10,myStringR,5);
        break;
  case 3:
        LCD_DrawString(xLevel,10,myStringY,4);
        break;
  }
  char num[3] = {myHex[(int)number/100],myHex[((int)number%100)/10],myHex[(int)number%10]};
  LCD_DrawString(xLevel,80,num,3);
}

void LCD_PrintN3(int type, double number, int xLevel){
  char myStringP[6] = {'P','i','t','c','h',':'};
  char myStringR[5] = {'R','o','l','l',':'};  
  char myStringY[4] = {'Y','a','w',':'}; 
  switch(type){
  case 1: 
        LCD_DrawString(xLevel,10,myStringP,6);
        break;
  case 2:
        LCD_DrawString(xLevel,10,myStringR,5);
        break;
  case 3:
        LCD_DrawString(xLevel,10,myStringY,4);
        break;
  }
  number = (number*(-2))+number;
  char num[4] = {'-',myHex[(int)number/100],myHex[((int)number%100)/10],myHex[(int)number%10]};
  LCD_DrawString(xLevel,80,num,4);
}


void ADXL_Init(char x_offset, char y_offset, char z_offset ){
  I2C_ByteWrite(ADXL345_Addr, ADXL345_POWER_CTL, 0x08);
  I2C_ByteWrite(ADXL345_Addr, ADXL345_DATA_FORMAT, 0x0B);
  
  I2C_ByteWrite(ADXL345_Addr,ADXL345_OFSX,x_offset);
  I2C_ByteWrite(ADXL345_Addr,ADXL345_OFSY, y_offset);
  I2C_ByteWrite(ADXL345_Addr,ADXL345_OFSZ,z_offset);
}

Axis_Values Get_Accel_Val(){ 
  
    double pitch, roll;
    Axis_Values aPR ;
    
    Cur_RawAX = (I2C_ByteRead(0xA6, 0x33) <<8)| (I2C_ByteRead(0xA6, 0x32) | 0);
    Cur_RawAY = (I2C_ByteRead(0xA6, 0x35)<<8)|(I2C_ByteRead(0xA6, 0x34) | 0);
    Cur_RawAZ = (I2C_ByteRead(0xA6, 0x37)<<8)|(I2C_ByteRead(0xA6, 0x36) | 0);	
        
    fXa =Cur_RawAX*0.0039065; + _xOff;
    fYa = Cur_RawAY*0.0039065; + _yOff;
    fZa = Cur_RawAZ*0.0039065; + _zOff;
    
    fXa = fXa*Accel_Alpha+(1-Accel_Alpha)*xa;
    fYa = fYa*Accel_Alpha+(1-Accel_Alpha)*ya;
    fZa= fZa*Accel_Alpha+(1-Accel_Alpha)*za;
    
    xa = fXa;    
    ya = fYa;
    za = fZa;   
    
    pitch = (atan2(fXa,sqrt(fYa*fYa+fZa*fZa)) * 180.0) / PI;
    roll = (atan2(fYa,(sqrt(fXa*fXa+fZa*fZa))) * 180.0) / PI;
    
    aPR.Pitch = pitch;
    aPR.Roll = roll;
    aPR.Yaw = 0;
    
    return aPR;
}

//Transmitter PWM change
void transmit_pwm(){
//    if(!MotorStarter && motorDuty<=MaxStartUpValue && baseValue > 180){
//      motorDuty +=1; 
//      TIM3->CCR1 = motorDuty;
//      TIM3->CCR2 = motorDuty;
//      TIM3->CCR3 = motorDuty;
//      TIM3->CCR4 = motorDuty;
//      MotorStarter = TRUE;
//    }
    if(baseValue > 180){
      if(motorDuty[0]<=MaxDuty){
        motorDuty[0] +=1;
        TIM3->CCR1 = motorDuty[0];
        if(motorDuty[0]>180) MotorStarter[0] = TRUE;
      }
      if(motorDuty[1]<=MaxDuty){
        motorDuty[1] +=1;
        TIM3->CCR2 = motorDuty[1];
        if(motorDuty[1]>180) MotorStarter[1] = TRUE;
      }
      if(motorDuty[2]<=MaxDuty){
        motorDuty[2] +=1;
        TIM3->CCR3 = motorDuty[2];
        if(motorDuty[2]>180) MotorStarter[2] = TRUE;
      }      
      if(motorDuty[3]<=MaxDuty){
        motorDuty[3] +=1;
        TIM3->CCR4 = motorDuty[3];
        if(motorDuty[3]>180) MotorStarter[3] = TRUE;
      }     
    }
    if(baseValue < 110){
      if(motorDuty[0]>=MinDuty){
        motorDuty[0] -=1;
        TIM3->CCR1 = motorDuty[0];
      }
      if(motorDuty[1]>=MinDuty){
        motorDuty[1] -=1;
        TIM3->CCR2 = motorDuty[1];
      }
      if(motorDuty[2]>=MinDuty){
        motorDuty[2] -=1;
        TIM3->CCR3 = motorDuty[2];
      }      
      if(motorDuty[3]>=MinDuty){
        motorDuty[3] -=1;
        TIM3->CCR4 = motorDuty[3];
      }     
    }
    
    return;
}

//PID functions
void Compute (){
    bool test = FALSE;

    double ROLL_error = Setpoint - roll_PID;
    double PITCH_error = Setpoint - pitch_PID;

    ROLL_ITerm += (ki * ROLL_error); 
    PITCH_ITerm += (ki * PITCH_error);
    
    if (ROLL_ITerm > outMax) ROLL_ITerm = outMax;   
    else if (ROLL_ITerm < outMin) ROLL_ITerm = outMin;  
    if (PITCH_ITerm > outMax) PITCH_ITerm = outMax;  
    else if (PITCH_ITerm < outMin) PITCH_ITerm = outMin;  

    double ROLL_dInput = (roll_PID - ROLL_LastInput);
    double PITCH_dInput = (pitch_PID - PITCH_LastInput); 
    
    //Compute PID output
    ROLL_Output_angle = kp * ROLL_error + ROLL_ITerm - kd * ROLL_dInput;
    PITCH_Output_angle = kp * PITCH_error + PITCH_ITerm - kd * PITCH_dInput;
    
    if (ROLL_Output_angle > outMax) ROLL_ITerm = outMax; 
    else if (ROLL_Output_angle < outMin) ROLL_ITerm = outMin;  
    if (PITCH_Output_angle > outMax) PITCH_ITerm = outMax; 
    else if (PITCH_Output_angle < outMin) PITCH_ITerm = outMin;  
                
    //Map to angle error to PWM value
   // ROLL_Output = ((120.0)/(500.0))* (ROLL_Output_angle+250)+120;
    
    ROLL_Output = ((120.0)/(500.0))* ROLL_Output_angle;
    PITCH_Output = ((120.0)/(500.0))* PITCH_Output_angle;

    //Remember variables
    ROLL_LastInput = roll_PID;
    PITCH_LastInput = pitch_PID;

    return;
}

void SetTunings(double Kp, double Ki, double Kd)
//try start with ki=0 , kp=0.39, kd=0.001
{
  // Ki, Kd units in 1/sec
   double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki*SampleTimeInSec;
   kd = Kd/SampleTimeInSec;
}

void SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime/(double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void PID_pwm(){
     int tempDuty1 = motorDuty[0] - (int)ROLL_Output - (int)PITCH_Output;
     int tempDuty2 = motorDuty[1]  - (int)ROLL_Output + (int)PITCH_Output;
     int tempDuty3 = motorDuty[2] + (int)ROLL_Output + (int)PITCH_Output;
     int tempDuty4 = motorDuty[3] + (int)ROLL_Output - (int)PITCH_Output ;
    
     if(tempDuty1>MaxDutyP) motorDuty[0] = MaxDutyP;
     else if(tempDuty1<MinDutyP && MotorStarter[0]) motorDuty[0] = MinDutyP;
     else motorDuty[0] = tempDuty1;
     
     if(tempDuty2>MaxDutyP) motorDuty[1] = MaxDutyP;
     else if(tempDuty2<MinDutyP && MotorStarter[1]) motorDuty[1] = MinDutyP;
     else motorDuty[1] = tempDuty2; 
     
     if(tempDuty3>MaxDutyP) motorDuty[2] = MaxDutyP;
     else if(tempDuty3<MinDutyP && MotorStarter[2]) motorDuty[2] = MinDutyP;
     else motorDuty[2] = tempDuty3;
     
     if(tempDuty4>MaxDutyP) motorDuty[3] = MaxDutyP;
     else if(tempDuty4<MinDutyP && MotorStarter[3]) motorDuty[3] = MinDutyP;
     else motorDuty[3] = tempDuty4;
     
     
     TIM3->CCR1 = motorDuty[0]; 
     TIM3->CCR2 = motorDuty[1]; 
     TIM3->CCR3 = motorDuty[2];  
     TIM3->CCR4 = motorDuty[3]; 

}  
  
//      motorDuty1 +=  -ROLL_Output - PITCH_Output;
//      TIM3->CCR1 = motorDuty1; //Positive Yaw
//      motorDuty2 += - ROLL_Output + PITCH_Output;
//      TIM3->CCR2 = motorDuty2; 
//      motorDuty3 +=  ROLL_Output + PITCH_Output;
//      TIM3->CCR3 = motorDuty3;  //Positive Yaw
//      motorDuty4 += ROLL_Output - PITCH_Output;
//      TIM3->CCR4 = motorDuty3; 
