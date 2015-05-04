  // PB7 = servo PWM HI
  // PB8 = servo PWM LO

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "pp-printf.h"
#include "biquad.h"
#include "servo.h"

#define GAIN_SHIFT 14

static void servo_adc_start();
static void servo_adc_stop();


struct bdf_state {
  int xd[10];
  int y;
};

static double notch1_center = 460.0;
static double notch2_center = 960.0;

struct servo_state {
    struct biquad lp_main;
    struct biquad notch1, notch2;

    struct bdf_state dfun;
    int gain_p, gain_d1;
    int gain_i;
    int bias;
    int integ;
};

static int bdf_update( struct bdf_state *st, int x )
{

  for(int i=9;i>=1;i--)
      st->xd[i] = st->xd[i-1];
  st->xd[0]= x;

  int y = 0;;
  y+=-4 * st->xd[8];
  y+=-3 * st->xd[7];
  y+=-2 * st->xd[6];
  y+=-1 * st->xd[5];
  y+=0 * st->xd[4];
  y+=1 * st->xd[3];
  y+=2 * st->xd[2];
  y+=3 * st->xd[1];
  y+=4 * st->xd[0];
  //y/=60;
  st->y=y;
  return y;
}

void bdf_init( struct bdf_state *st )
{
  int i;
  for(i=0;i<10;i++)
    st->xd[i] = 0;
}



static struct servo_state servo;
static volatile int servo_setpoint = 2400;

static volatile int adc_samples = 0;
static volatile struct servo_log_entry servo_log[SERVO_LOG_SIZE];
static volatile int servo_log_start = 0, servo_log_count = SERVO_LOG_SIZE;

static inline void servo_drive_set(int value)
{
  if( value < 0 )
  {
      TIM4->CCR3 = -value;
      TIM4->CCR2 = 0;
  } else {
      TIM4->CCR3 = 0;
      TIM4->CCR2 = value;
  }
}



static inline void servo_update(int x)
{
  int err = x - servo_setpoint;

  if(servo.integ < 1000000000 && err > 0)
    servo.integ += err;

  else if(servo.integ > -1000000000 && err < 0)
    servo.integ += err;

  int term_i = servo.gain_i * servo.integ;
  int term_p = servo.gain_p * err;
  int term_d1 = servo.gain_d1 * bdf_update(&servo.dfun, x) ;

  int y = servo.bias + ( (term_p + term_d1 + term_i ) >> GAIN_SHIFT );


  //y = biquad_update(&servo.notch1, y);
  //y = biquad_update(&servo.notch2, y);

  //y = 10000;

  //y = biquad_update(&servo.lp_main, y);

  if( y > 1000 )
    y= 1000;

  if (y< -1000)
    y = -1000;

//y=-y;

  servo_drive_set ( y );

    if(adc_samples > servo_log_start && servo_log_count < SERVO_LOG_SIZE)
    {
      servo_log[servo_log_count].setpoint = servo_setpoint;
      servo_log[servo_log_count].error = x;
      servo_log[servo_log_count].y = y;
      servo_log_count ++;
    }
}

static volatile uint16_t *test_acq_buf;
static volatile int test_acq_count = 0;
#define ADC_DMA_BUF_SIZE 32

static volatile uint16_t adc_dma_buf[ADC_DMA_BUF_SIZE];

void DMA2_Stream0_IRQHandler()
{
  int sum = 0, i;

  servo_adc_stop();

  GPIO_SetBits(GPIOB, GPIO_Pin_6);
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);

  adc_samples++;

  DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

  servo_adc_start();

  for(i = 0; i < ADC_DMA_BUF_SIZE; i++)
    sum += adc_dma_buf[i];

  sum /= ADC_DMA_BUF_SIZE;

  servo_update(sum);


  if(test_acq_count)
  {
    *test_acq_buf++ = sum;
    test_acq_count--;
  }
}


void servo_set_setpoint(int target)
{
  servo_setpoint = target;
}




void dump_buf()
{
  pp_printf("ADC Buffer Dump : \n\r");
  int i;
  for(i=0;i<ADC_DMA_BUF_SIZE;i++)
    pp_printf("%d ", adc_dma_buf[i]);
  pp_printf("\n\r");

}
static void servo_adc_init()
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  /* Enable peripheral clocks *************************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

DMA_InitTypeDef       DMA_InitStructure;

  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adc_dma_buf;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = ADC_DMA_BUF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);


  /* ADCs configuration ------------------------------------------------------*/
  /* Configure ADC Channel10, 11, 12 pin as analog input */


  GPIO_InitTypeDef GPIO_InitStructure;
  /* ADC Channel 10 -> PC0
     ADC Channel 11 -> PC1
     ADC Channel 12 -> PC2
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // sample rate: 84 MHz / 8 / (480 + 12) = 21.341 kHz

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels 10, 11 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

/* Enable DMA */
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

  servo_adc_start();
}

static void servo_adc_stop()
{
  ADC_DMACmd(ADC1, DISABLE);
  ADC_Cmd(ADC1, DISABLE);
}


static void servo_adc_start()
{

  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
}



void servo_pwm_init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // TIM4/PWM3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);



  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // TIM4/PWM2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

   /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

/* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);


  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1200; // end-of-pwm sampling point for the ADC
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);

}

void servo_init()
{
    biquad_init(&servo.lp_main, BIQUAD_TYPE_LOWPASS,SERVO_SAMPLE_RATE, 200.0,  1.0, 0.0);
    biquad_init(&servo.notch1, BIQUAD_TYPE_PEAK, SERVO_SAMPLE_RATE, notch1_center,  10.0, -20.0);
    biquad_init(&servo.notch2, BIQUAD_TYPE_PEAK, SERVO_SAMPLE_RATE, notch2_center,  10.0, -20.0);
    bdf_init( &servo.dfun );


    servo.gain_p = 4 * 15 * 270;//1.0 * 255; //0.8*255;
    servo.gain_d1 =  10000;//4*15 * 900; //3000;
    servo.gain_i = 2;//2;//4;
    servo.bias = 320;
    servo.integ = 0;

    servo_pwm_init();
    servo_adc_init();


}


void servo_start_logging( int offset )
{
  pp_printf("[servo-log] started logging.\n\r" );
  servo_log_start = offset;
  adc_samples = 0;
  servo_log_count = 0;
}

void servo_stop_logging( struct servo_log_entry **log, int *n_samples )
{
  *log = (struct servo_log_entry*)servo_log;
  *n_samples = servo_log_count;
  pp_printf("[servo-log] %d samples logged.\n\r", servo_log_count);
}

void test()
{
 for(;;)

  pp_printf("adc_samples %d\n\r", adc_samples);
}

void servo_test_acquisition(int n_samples, int averaging, uint16_t *buf)
{
 #if 0
  ADC_InitTypeDef ADC_InitStructure;


  ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels 10, 11 configuration */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);

  ADC_Cmd(ADC1, ENABLE);
  ADC_SoftwareStartConv(ADC1);

  int i, j, sum;

  for(i = 0; i < n_samples; i++)
  {
      sum = 0;
      for(j =0; j < averaging; j++)
      {
        while(! ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) );
        ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
        sum+=ADC_GetConversionValue(ADC1);
      }
      buf[i] = sum / averaging;
  }
  #endif
  __disable_irq();
  test_acq_count = n_samples;
  test_acq_buf = buf;
  __enable_irq();

  while(test_acq_count);


}

void test_servo_dma()
{
  uint16_t buf[1024];
  int i;
  servo_test_acquisition(1024, 1, buf);

  for(i=0;i<1024;i++)
    pp_printf("%d ", buf[i]);

  pp_printf("\n\rDone!\n\r");

 /*for(;;)
  {
    FlagStatus f1 = DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0);
    pp_printf("s  %d\n\r", adc_samples);
  }*/

}