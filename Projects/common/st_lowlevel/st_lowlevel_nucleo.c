#include "S2LP_SDK_Util.h"
#include "SDK_EVAL_Config.h"
#include <string.h>
#include "st_lowlevel.h"
#include "MCU_Interface.h"
#include "sigfox_retriever.h"
#include "st_lowlevel_utils.h"

#define ST_TRUE  1
#define ST_FALSE 0

#if defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F0XX_NUCLEO)
#define IRQ_PRIORITY 0x00
#else
#define IRQ_PRIORITY 0x0A
#endif

static RTC_HandleTypeDef RtcHandler={.Instance=RTC};
StatusBytes SdkEvalSpiRaw(uint8_t cNbBytes, uint8_t* pInBuffer, uint8_t* pOutBuffer, uint8_t can_return_before_tc);

static volatile uint8_t rtc_irq=0, rtc_in_use=0, notify_end=0;
static volatile uint8_t low_power=1;
static volatile uint32_t next_rtc_wakeup=0;
static volatile int16_t rtc_presc=2375;

static volatile uint32_t n_intermediate_tim_irq=0;


void Appli_Exti_CB(uint16_t GPIO_Pin);

static void Configure_RTC_Clock(void)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
#ifndef USE_STM32L0XX_NUCLEO
  __HAL_RCC_PWR_CLK_ENABLE();
#endif
  HAL_PWR_EnableBkUpAccess();

  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  

  /* Enable RTC Clock */ 
  __HAL_RCC_RTC_ENABLE(); 
  
  HAL_NVIC_SetPriority(STM32_RTC_IRQn, 0x01, 0);
  HAL_NVIC_EnableIRQ(STM32_RTC_IRQn);
}




static void setGpioLowPower(void)
{
 
  // complete init of GPIOs
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_0);   // IRQ
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = GPIO_PIN_All & (~GPIO_PIN_8) & (~GPIO_PIN_1) & (~GPIO_PIN_9);// SDN , CS S2-LP and CS E2PROM
  
#if defined(USE_STM32F0XX_NUCLEO) || defined(USE_STM32F4XX_NUCLEO)
  GPIO_InitStructure.Pin &= (~GPIO_PIN_2) & (~GPIO_PIN_3);
#endif
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  __GPIOD_CLK_DISABLE();__GPIOB_CLK_DISABLE();

  
}

static void setGpioRestore(void)
{
  STM32_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode =    GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate= EEPROM_SPI_PERIPH_MISO_AF;
  GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.Mode =    GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Alternate= EEPROM_SPI_PERIPH_MISO_AF;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

}


void STM32_RTC_IRQHandler(void)
{
  Configure_RTC_Clock();
  
  HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandler);
  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
  
  
  if(next_rtc_wakeup==0)
  {
    rtc_irq=1;
    rtc_in_use=0;
    if(notify_end)
    {
      ST_MANUF_Timer_CB();
    }
  }
  else
  {
    ST_LOWLEVEL_TimerStart(next_rtc_wakeup);
    n_intermediate_tim_irq++;
  }
}



void ST_LOWLEVEL_Shutdown(uint8_t value)
{
  if(value==ST_TRUE)
    SdkEvalEnterShutdown();
  else
    SdkEvalExitShutdown();
}




void ST_LOWLEVEL_Delay(uint32_t delay_ms)
{
  if(rtc_in_use || delay_ms<10)
  {
    TIM_HandleTypeDef Tim2_Handler={.Instance=TIM2};
    Tim2_Handler.Init.Prescaler         = 16000-1;
    Tim2_Handler.Init.Period            = delay_ms-1;
    Tim2_Handler.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    Tim2_Handler.Init.CounterMode       = TIM_COUNTERMODE_UP;
    
    /* disable the timer */
    __HAL_TIM_DISABLE(&Tim2_Handler);
    /* Configure the timer in update mode */
    __HAL_TIM_DISABLE_IT(&Tim2_Handler, TIM_IT_UPDATE);
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* Init the time base structure */
    HAL_TIM_Base_Init(&Tim2_Handler);
    __HAL_TIM_CLEAR_FLAG(&Tim2_Handler, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start(&Tim2_Handler);

      
    while(!__HAL_TIM_GET_FLAG(&Tim2_Handler, TIM_FLAG_UPDATE));
    __HAL_TIM_CLEAR_FLAG(&Tim2_Handler, TIM_FLAG_UPDATE);

    
    HAL_TIM_Base_Stop(&Tim2_Handler);
    __HAL_RCC_TIM2_CLK_DISABLE();
  }
  else
  {
    Configure_RTC_Clock();
    notify_end=0;
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
    __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
    n_intermediate_tim_irq=0;
    rtc_in_use=1;
    rtc_irq=0;
    next_rtc_wakeup=0;
    HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler,delay_ms*rtc_presc/1000,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
    while(!rtc_irq)
    {
      ST_LOWLEVEL_WaitForInterrupt();
    }
    rtc_irq=0;
    HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
    rtc_in_use=0;
  }
}



void ST_LOWLEVEL_SpiRaw(uint8_t n_bytes, uint8_t* in_buffer, uint8_t* out_buffer, uint8_t can_return_bef_tx)
{
  /* in this implementation we are not interested in the value of the can_return_bef_tx flag.
    We always pass 0 to the SdkEvalSpiRaw so that the CPU will wait the DMA for the end of transfer. */
  SdkEvalSpiRaw(n_bytes,in_buffer,out_buffer,0);
}

void ST_LOWLEVEL_Encrypt(uint8_t *encrypted_data, uint8_t *data_to_encrypt, uint8_t data_len)
{
  enc_utils_encrypt(encrypted_data, data_to_encrypt, data_len);
}



void ST_LOWLEVEL_TimerStart(uint32_t time_duration_ms)
{
  Configure_RTC_Clock();
  notify_end=1;
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
  n_intermediate_tim_irq=0;
  rtc_in_use=1;
  
  if(time_duration_ms>25000)
  {
    next_rtc_wakeup=time_duration_ms-25000;
    time_duration_ms=25000;
  }
  else
  {
    next_rtc_wakeup=0;
  }
  HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler,time_duration_ms*rtc_presc/1000,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
}


uint32_t ST_LOWLEVEL_TimerGetExpMs(void)
{
#ifdef FOR_ARIB
  static uint32_t tick;
  tick=RtcHandler.Instance->TR*1000+(1000*(RtcHandler.Instance->PRER & 0xFF)-RtcHandler.Instance->SSR)/((RtcHandler.Instance->PRER & 0xFF)+1);
  return (n_intermediate_tim_irq*25000+tick);
#else
  return 0;
#endif
}

void ST_LOWLEVEL_UTILS_TimerCalibration(uint16_t duration_ms)
{
  TIM_HandleTypeDef  Tim2_Handler={.Instance=TIM2};
  uint16_t c;
  Configure_RTC_Clock();
  notify_end=1;
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RtcHandler, RTC_FLAG_WUTF);
  __HAL_RTC_CLEAR_FLAG(RTC_EXTI_LINE_WAKEUPTIMER_EVENT);
  
  next_rtc_wakeup=0;

  SdkEvalTimersTimConfig(&Tim2_Handler,16000-1,65535-1);
  __HAL_TIM_DISABLE_IT(&Tim2_Handler, TIM_IT_UPDATE);
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  HAL_TIM_Base_Start(&Tim2_Handler);
  
  HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandler,rtc_presc*duration_ms/1000,RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  while(!rtc_irq);
  c=Tim2_Handler.Instance->CNT;
  rtc_irq=0;
  HAL_TIM_Base_Stop(&Tim2_Handler);

  rtc_presc=duration_ms*rtc_presc/c;
  
} 


void ST_LOWLEVEL_TimerStop(void)
{
  Configure_RTC_Clock();
  HAL_RTCEx_DeactivateWakeUpTimer(&RtcHandler);
  rtc_in_use=0;
}

uint16_t ST_LOWLEVEL_NVMRead(uint32_t offset)
{
  /* The implementation of NVM Read has changed to refer the external EEPROM rather than the internal E2PROM of the MCU */
  /* return (uint16_t)(*((uint32_t*)(STM32_EEPROM_BASE+offset))); */
  
  uint32_t ret;
  
  /* read the data from the extarnal EEPROM */
  EepromRead(0x600+offset, 4, (uint8_t*)&ret);
  return (uint16_t)ret;
}
 
void ST_LOWLEVEL_NVMWrite(uint32_t offset, uint32_t value)
{ 
  /* The implementation of NVM Write has changed to refer the external EEPROM rather than the internal E2PROM of the MCU */
  /*  
  HAL_FLASHEx_DATAEEPROM_Unlock();
  HAL_FLASHEx_DATAEEPROM_Program(STM32_TYPEPROGRAM_WORD, STM32_EEPROM_BASE+offset, value);
  HAL_FLASHEx_DATAEEPROM_Lock();
  */

  
  /* store the data in the external EEPROM */
  uint32_t value_r;
  EepromWrite(0x600+offset, 4, (uint8_t*)&value);
  ST_LOWLEVEL_Delay(6);
  EepromRead(0x600+offset, 4, (uint8_t*)&value_r);
  
  if(value!=value_r)
    while(1);
}

/* trigger 1: rising, 0: falling*/
void ST_LOWLEVEL_GpioIRQ(uint8_t pin, uint8_t new_state, uint8_t trigger)
{ 
  M2SGpioPin GpioPin;
  
  switch(pin)
  {
  case 0:
    GpioPin=M2S_GPIO_0;
    break;
  case 1:
    GpioPin=M2S_GPIO_1;
    break;    
  case 2:
    GpioPin=M2S_GPIO_2;
    break;
  case 3:
    GpioPin=M2S_GPIO_3;
    break;
  }
  
  __HAL_GPIO_EXTI_CLEAR_IT(SdkEvalGpioGetPin(GpioPin));
  if(new_state)
  {
    SdkEvalM2SGpioInit(GpioPin, M2S_MODE_EXTI_IN);
    SdkEvalM2SGpioInterruptCmd(GpioPin, IRQ_PRIORITY, 0x00, ENABLE);
    if(trigger)
    {
      SdkEvalM2SGpioTriggerRising(GpioPin,ENABLE);
      SdkEvalM2SGpioTriggerFalling(GpioPin,DISABLE);
    }
    else
    {
      SdkEvalM2SGpioTriggerRising(GpioPin,DISABLE);
      SdkEvalM2SGpioTriggerFalling(GpioPin,ENABLE);
    }
  }
  else
  {
    SdkEvalM2SGpioInterruptCmd(GpioPin, IRQ_PRIORITY, 0x00, DISABLE);
    SdkEvalM2SGpioInit(GpioPin, M2S_MODE_GPIO_IN);
  }
}


#ifdef USE_STM32L0XX_NUCLEO
void ST_LOWLEVEL_UTILS_SetSysClock(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_4;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
  }
}
#elif USE_STM32F0XX_NUCLEO
void ST_LOWLEVEL_UTILS_SetSysClock(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Select HSI48 Oscillator as PLL source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV6;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
  }

  /* Select PLL as system clock source and configure the HCLK and PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
  }
  
}
#elif USE_STM32F4XX_NUCLEO
void ST_LOWLEVEL_UTILS_SetSysClock(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
  }  
}
#else
void ST_LOWLEVEL_UTILS_SetSysClock(void)
{

  __IO uint32_t StartUpCounter = 0, HSIStatus = 0, HSEStatus = 0;
    

  /* Enable HSI */
  RCC->CR |= ((uint32_t)RCC_CR_HSION);
   
  /* Wait till HSI is ready and if Time out is reached exit */
  do
  {
    HSIStatus = RCC->CR & RCC_CR_HSIRDY;
  } while((HSIStatus == 0) && (StartUpCounter != 0x5000));

  if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
  {
    HSIStatus = (uint32_t)0x01;
  }
  else
  {
    HSIStatus = (uint32_t)0x00;
  }
    
  if (HSIStatus == (uint32_t)0x01)
  {
    /*  PLL configuration */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL |RCC_CFGR_PLLDIV));
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV4);
  //RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLDIV3);
  }
  
  else
  {
    /* If HSI fails to start-up, the application will have wrong clock
    configuration. User can add here some code to deal with this error */
  }
  
  /* Enable 64-bit access */
  FLASH->ACR |= FLASH_ACR_ACC64;
  
  /* Enable Prefetch Buffer */
  FLASH->ACR |= FLASH_ACR_PRFTEN;
  
  /* Flash 1 wait state */
  FLASH->ACR |= FLASH_ACR_LATENCY;
  
  /* Power enable */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  
  /* Select the Voltage Range 1 (1.8 V) */
  PWR->CR = PWR_CR_VOS_0;
  
  /* Wait Until the Voltage Regulator is ready */
  while((PWR->CSR & PWR_CSR_VOSF) != RESET)
  {
  }
  
  /* HCLK = SYSCLK /1*/
  RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
  
  /* PCLK2 = HCLK /1*/
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
  
  /* PCLK1 = HCLK /1*/
  RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;
  


  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;
  
  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }
  
  /* Select PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
  
  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }

}

#endif

uint8_t s2lp_irq_raised=0;

void ST_LOWLEVEL_WaitForInterrupt(void)
{
  
  if(low_power)
  {
    setGpioLowPower();
    
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFI);
    
    ST_LOWLEVEL_UTILS_SetSysClock();
    
    setGpioRestore();
   
  }
  
  if(s2lp_irq_raised)
  {
    ST_MANUF_S2LP_Exti_CB();
    s2lp_irq_raised=0;
  }
}



void ST_LOWLEVEL_UTILS_LowPower(uint8_t en)
{
  //low_power=en;
  low_power = 0;
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==GPIO_PIN_0)
  {    
    s2lp_irq_raised=1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
  }
  else
  {    
    Appli_Exti_CB(GPIO_Pin);
  }
}




int32_t ST_LOWLEVEL_GetTemperature(void)
{
  return 0;
}

int32_t ST_LOWLEVEL_GetVoltage(void)
{
  return 0;
}




/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
