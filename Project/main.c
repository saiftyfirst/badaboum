/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "myfunctions.c"
bool GyroTest = FALSE;
bool Read = FALSE;


int main(void)
{
  SystemInit();
  STM3210E_LCD_Init(); 
  I2C_EE_Init(); 
//ADXL_Init(); 
  Gyro_Init_Test(-0.18891-0.04382, -0.09454-0.05530, -0.31720-0.04951); 
  ADXL_Init(-0.023, 0, 0.03577027);  
  
   
  Init_Pin(GPIO_Pin_6, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIOA, RCC_APB2Periph_GPIOA);
  Init_Pin(GPIO_Pin_7, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIOA, RCC_APB2Periph_GPIOA); //CH2
  Init_Pin(GPIO_Pin_0, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIOB, RCC_APB2Periph_GPIOB); //CH3
  Init_Pin(GPIO_Pin_1, GPIO_Mode_AF_PP, GPIO_Speed_50MHz, GPIOB, RCC_APB2Periph_GPIOB); //CH4  
  
  
  
  //Main Loop
  Init_Timer(10,20,TIM8,RCC_APB2Periph_TIM8,1); 
  Init_Timer_MS(TIM8,TIM5,199,40,RCC_APB1Periph_TIM5,TIM_TS_ITR3);  
  TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
  Interrupt_Attach(TIM5_IRQn,1);
  
  //PWM waves
  Init_Timer(100,20,TIM4,RCC_APB1Periph_TIM4,1); 
  Init_Timer_MS(TIM4,TIM3,999,motorDuty[0],RCC_APB1Periph_TIM3,TIM_TS_ITR3);  
  
  //Transceiver
  Init_Pin(GPIO_Pin_1, GPIO_Mode_IPD, GPIO_Speed_50MHz, GPIOA, RCC_APB2Periph_GPIOA); 
  Set_EXT_Channel(EXTI_Line1, GPIO_PortSourceGPIOA,  GPIO_PinSource1, EXTI_Trigger_Rising_Falling);
  Interrupt_Attach(EXTI1_IRQn,2);
  
  //Transciever
  Init_Timer(100,20,TIM1,RCC_APB2Periph_TIM1,1);
  Init_Timer_MS(TIM1,TIM2,999,500,RCC_APB1Periph_TIM2,TIM_TS_ITR0); 
  
  
  Axis_Values gDPS;
  double pitchG,rollG,yawG;
  bool changeSel =FALSE;
  //SetTunings(0.39,0.0,0.001);
  SetTunings(0.39,0.0,0.001);
 
  while(1){
    
   // TIM3->CCR1 = motorDuty;
      
    if(GyroTest){
   
      gDPS = Get_Gyro_Val();  
      pitch_PID = gDPS.Pitch;
      roll_PID = gDPS.Roll;
      //transmit_pwm();
      

      transmit_pwm();
       
//      if(pitch_PID > 0){
//        motorDuty3 = motorDuty3 + (int)((pitch_PID/250.0)*60.0);
//        TIM3->CCR3 = motorDuty3;
//      }
      
      Compute(); 
     
      PID_pwm();
      
      LCD_Clear();
      
      LCD_PrintP3(2, roll_PID, 1 );
      LCD_PrintP3(1, motorDuty[0],  3);
      LCD_PrintP3(3, baseValue, 6); 
      
      GyroTest = FALSE;   
    }
  }
}