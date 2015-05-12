#include "stm32f4xx.h"
#include "pp-printf.h"

#include "common.h"

#include <limits.h>

static float esc_target_speed = 0.0;
static uint32_t esc_update_count_last = 0;

int esc_pwm_init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  uint16_t PrescalerValue = 0;

  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);


  /* GPIOC Configuration: TIM3 CH1 (PC6), TIM3 CH2 (PC7), TIM3 CH3 (PC8) and TIM3 CH4 (PC9) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);

  PrescalerValue = (uint16_t) ((SystemCoreClock ) / 2100000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 21000 * 25 / 10;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  PrescalerValue = 1;

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 100;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);

  return 0;
}


void esc_throttle_set(float value)
{
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  int v = 2100 + (int)(value * 2100.0);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = v;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

}

static void EXTILine12_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line12;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


struct pi_control
{
  float kp, ki;
  float integrator;
  float y_bias, y_min, y_max;
  float x0;
};


void pi_init( struct pi_control *pi )
{
  pi->integrator = 0.0;
  pi->x0 = 0.0;
}

float pi_update( struct pi_control *pi, float x )
{
  pi->integrator += x;
  float y = pi->y_bias + pi->integrator * pi->ki + pi->kp * x;
  if(y > pi->y_max) y= pi->y_max;
  if(y < pi->y_min) y= pi->y_min;
  pi->x0 = x;

  return y;
}

struct pi_control esc_pi;

void esc_enable_power(int enable)
{
  if(enable)
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
  else
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
}

void esc_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    EXTILine12_Config();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 0xffffffff;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);

    esc_pi.ki = 0.00005;
    esc_pi.kp = 0.05;
    esc_pi.y_min = 0.0;
    esc_pi.y_max = 1.0;
    esc_pi.y_bias = 0.5;

    pi_init(&esc_pi);

    esc_pwm_init();
    pp_printf("PWM initialized\n\r");
    esc_throttle_set(0);
    delay(100);
    esc_enable_power(1);
    delay(2000);
    pp_printf("ESC initialized.\n\r");
}

int last_esc_tics_valid = 0;
uint32_t last_esc_tics = 0;

#define PULSE_FILTER_SIZE 32
#define PULSE_FILTER_MASK 0x1f

volatile int esc_pulse_filter [ PULSE_FILTER_SIZE ];
volatile int esc_wpos = 0;
volatile int esc_expected_value = 0;

volatile uint32_t last_tacho_ticks = 0;
volatile uint32_t esc_last_origin = 0;
volatile int esc_marks_per_turn = 0;
volatile int esc_mark_count = 0;

void esc_pulse_handler()
{
  uint32_t tics = TIM_GetCounter(TIM2);

  if(last_esc_tics_valid)
  {
    int delta;

    if ( tics < last_esc_tics )
      delta = last_esc_tics - tics;
    else
      delta = tics - last_esc_tics;

    if(delta > 4000) // ignore noise
    {
      esc_pulse_filter [ (esc_wpos++) & PULSE_FILTER_MASK ] = delta;

      if (delta > 5 * esc_expected_value / 4) // got full rotation marker?
      {
        esc_marks_per_turn = esc_mark_count;
        esc_mark_count = 0;
      }

      if(delta > esc_expected_value / 3)
      {
        esc_mark_count ++;
        esc_last_origin = tics;
      }
    }
  }

  last_tacho_ticks = get_ticks_count();

  last_esc_tics_valid = 1;
  last_esc_tics = tics;
}

void bsort(int *x, int n)
{
  int i,j;
  for(i=0;i<n;i++)
    for(j=0;j<n;j++)
      if(x[i]>x[j])
      {
        int tmp = x[i];
        x[i] = x[j];
        x[j] = tmp;
      }
}

int esc_compute_outliers()
{
  int pf[PULSE_FILTER_SIZE];

  memcpy(pf, esc_pulse_filter, PULSE_FILTER_SIZE * sizeof(int));
  bsort(pf, PULSE_FILTER_SIZE);
  int median = pf[PULSE_FILTER_SIZE/2];
    int avg = 0, n = 0, i;



  for(i=0;i<PULSE_FILTER_SIZE;i++)
  {
    int ratio = (100 * abs(pf[i] - median)) / median;

    if( ratio < 5 )
    {
      avg += pf[i];
      n++;
    }
  }



  __disable_irq();

  if(n)
  {
    avg/=n;
    //pp_printf("avg %d n %d\n\r", avg, n );
    esc_expected_value = avg;
  } else {
    esc_expected_value = 10000000;
  }

  __enable_irq();


}



int esc_get_speed()
{
  esc_compute_outliers();

  return esc_expected_value;
}

#define MARKS_PER_TURN 179

float esc_get_speed_rps()
{
  esc_compute_outliers();


  int dt = get_ticks_count() - last_tacho_ticks;
  if(dt > 10)
    return 0.0;

  //pp_printf("dt %d\n\r", dt);

  if(esc_expected_value == 0)
    return 0.0;

  float v = 1.0f / ( (float)MARKS_PER_TURN * (float) esc_expected_value / (float) SystemCoreClock );
  return v;
}




void esc_set_speed ( float setpoint_rps )
{
  esc_target_speed = setpoint_rps;
}

#define ESC_UPDATE_PERIOD_MS 10 // ms

void esc_control_update()
{
#if 1
  uint32_t ticks = get_ticks_count();

  if(ticks - esc_update_count_last >= ESC_UPDATE_PERIOD_MS)
  {
    esc_update_count_last = ticks;

    float err = esc_target_speed - esc_get_speed_rps();

    float y = pi_update(&esc_pi, err);

    //pp_printf("speed %d err %d throttle %d orig %d pos %d\n\r", (int)esc_get_speed_rps(), (int)(1000.0*err), (int)(1000.0*y), esc_last_origin, esc_get_radial_position());

    //if(esc_target_speed>0)
    esc_throttle_set(y);
  }
#endif

  //esc_throttle_set(0.3);
}



void ldrive_init()
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOC, GPIO_Pin_5);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);

    esc_init();
}

void ldrive_step( int dir )
{
    if(dir)
      GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    else
      GPIO_SetBits(GPIOC, GPIO_Pin_9);

    for(volatile int i =0;i<1000;i++);

    GPIO_SetBits(GPIOC, GPIO_Pin_5);
    for(volatile int i =0;i<1000;i++);
    GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}



/*void test_tacho()
{
  pp_printf("TestTacho:\n\r");
  for(;;)
    {
      if(esc_expected_value > 0)
        pp_printf("Exp: %d\n\r", esc_expected_value);

      esc_control_update();
      delay(1);
    }
}*/

int esc_get_radial_position()
{
  uint32_t marks;
  uint32_t origin;
  uint32_t cnt;
  __disable_irq();
  marks = esc_mark_count;
  origin = esc_last_origin;
  cnt = TIM_GetCounter(TIM2);
  __enable_irq();


  return (esc_mark_count - 1) * 200 + ( 200LL * ((int64_t)cnt - origin) / esc_expected_value );
}