#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "pp-printf.h"
#include "biquad.h"
#include "usart.h"
#include "servo.h"
#include "rpc.h"
#include "motors.h"

extern void pwm_test();
void fet_charge(int on);
void fet_break(int on);
int is_touchdown();
uint32_t get_ticks_count();
void hv_init();
void delay(int ms);


void clock_info()
{
    RCC_ClocksTypeDef rcc;
    RCC_GetClocksFreq(&rcc);

    pp_printf("SWS: %d\n\r", rcc.sws);
    pp_printf("SYSCLK: %d Hz\n\r", rcc.SYSCLK_Frequency);
    pp_printf("AHB: %d Hz\n\r", rcc.HCLK_Frequency);
    pp_printf("APB1: %d Hz\n\r", rcc.PCLK1_Frequency);
    pp_printf("APB2: %d Hz\n\r", rcc.PCLK2_Frequency);
}


int wait_key()
{
  while(!usart_poll());
  return usart_rx_char();
}

#if 0
void test_esc()
{
  //esc_init();
    motors_init();

    //esc_throttle_set(0.3);
    //throttle_set(250);
    esc_set_speed(20.0);

    int n = 0, dn = 1;


//    arc_test();

    //esc_throttle_set(0.25);


    for(;;)
    {
      //EXTI_GenerateSWInterrupt(EXTI_Line3);
      esc_control_update();
      //pp_printf("Pulses: %d cnt %d\n\r", pcnt,  TIM_GetCounter(TIM2) );
      //pp_printf("n %d dn %d ESC Speed %d %d\n\r", n, dn, esc_get_speed(), (int) esc_get_speed_rps() );
      delay(10);

  #if 0
  n+=dn;

      if(dn > 0)
        head_step(1);
      else
        head_step(0);

      if( n == 100 || n == 0)
      {
        dn = -dn;
      }
      #endif

    }

}

void test_gpio_pin( void *gpio, int pin )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = pin; // TIM4/PWM3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(gpio, &GPIO_InitStructure);

  for(;;)
  {
    GPIO_SetBits(gpio, pin);
    delay(1);
    GPIO_ResetBits(gpio, pin);
    delay(1);
  }

}

int setpoint = 1000;

#if 1

void handle_kb()
{



    if (usart_poll())
    {
      int c = usart_rx_char();
      int new_setting = 1;

      switch(c)
      {
       /*   case 'z': if(notch1_center > 100) notch1_center -= 3.0; break;
          case 'a': if(notch1_center < 2000) notch1_center += 3.0;break;
          case 'c': if(notch1_center > 100) notch2_center -= 3.0; break;
          case 'd': if(notch1_center < 2000) notch2_center += 3.0;break;
          case 's': servo.gain_d1 += 5; break;
          case 'x': servo.gain_d1 -= 5; break;*/
          case 'f': setpoint += 5; break;
          case 'v': setpoint -= 5; break;
          default:
        new_setting = 0;
      }

    if(new_setting)
    {
      pp_printf("sp: %d\n\r", setpoint);
      servo_set_setpoint(setpoint);
    }
  }

}
#endif

#endif

uint16_t buf[8192];



void cmd_adc_test(struct rpc_request *rq)
{
  int n_samples = rpc_pop_int32(rq);
  int n_avg = rpc_pop_int32(rq);

  servo_test_acquisition(n_samples, n_avg, buf);
  rpc_answer_send(RPC_ID_ADC_TEST, 2 * n_samples, buf);

}

void cmd_esc_set_speed(struct rpc_request *rq)
{
  float speed = (float) rpc_pop_int32(rq) / 1000.0;
  pp_printf("SetSpeed: %d\n", (int)speed);
  esc_set_speed( speed );
}

void cmd_esc_get_speed(struct rpc_request *rq)
{
  float speed = esc_get_speed_rps( );
  int speed_int = (int) ( speed * 1000.0 );

  rpc_answer_send(RPC_ID_GET_ESC_SPEED, 4, &speed_int);
}


void cmd_servo_response(struct rpc_request *rq)
{
  int init_setpoint = rpc_pop_int32(rq);
  int target_setpoint = rpc_pop_int32(rq);
  struct servo_log_entry *log;
  int log_samples;

  servo_set_setpoint(init_setpoint);
  servo_start_logging( 0 );
  delay(200);
  int i;
  for(i=0;i<3;i++)
  {
  delay(20);
  servo_set_setpoint(target_setpoint);
  delay(20);
  servo_set_setpoint(init_setpoint);
  }
  delay(200);
  servo_stop_logging(&log, &log_samples);

//  pp_printf("log has %d samples\n\r", log_samples);
  rpc_answer_send(RPC_ID_SERVO_RESPONSE, log_samples * sizeof(struct servo_log_entry), log);
}



struct timeout {
  uint32_t last;
  uint32_t period;
};

void tmo_init(struct timeout *tmo, uint32_t period)
{
  tmo->last = get_ticks_count();
  tmo->period = period;
}

int tmo_hit(struct timeout *tmo)
{
  uint32_t t =  get_ticks_count();
  if(t - tmo->last >= tmo->period)
  {
    tmo->last = t;
    return 1;
  }
  return 0;
}

void main_loop()
{
  struct rpc_request rq;
  struct timeout keepalive;

  tmo_init(&keepalive, 1000);
  while (1)
  {
    esc_control_update();

    if(tmo_hit(&keepalive))
      pp_printf("Heartbeat!\n\r");

    if(!rpc_request_get(&rq))
      continue;


    switch(rq.id)
    {
      case RPC_ID_ADC_TEST: cmd_adc_test(&rq); break;
      case RPC_ID_GET_ESC_SPEED: cmd_esc_get_speed(&rq); break;
      case RPC_ID_SET_ESC_SPEED: cmd_esc_set_speed(&rq); break;
      case RPC_ID_SERVO_RESPONSE: cmd_servo_response(&rq); break;

      default: break;
    }
  }
}

int main(void)
{
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000);


    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    usart_init();
    //clock_info();
    servo_init();
    esc_init();
    esc_set_speed(10.0);

    //test_tacho();

    //test_servo_dma();
    main_loop();



    for(;;)
    {
      FlagStatus f1 = DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0);
      if(f1)
      {
        dump_buf();
        break;
      }



    }

    for(;;);
    //main_loop();
#if 0

//    pwm_init();

  //  hv_init();


  //  fet_charge(0);
   // fet_break(0);

    //servo_drive_set(1000);

    servo_set_setpoint(2500);
    //servo_set_setpoint( 1000 );


   /* for(;;)
    {
      adc_samples = 0;
      delay(1000);
      pp_printf("samples: %d\n\r", adc_samples);
    }*/

    //for(;;)
    {
      //pp_printf("s = start response test\n\r");
      //while(wait_key() != 's');

      pp_printf("Response test...\n\r");

      //test();
      servo_start_logging(0);

      delay(100);

      for(;;)
        handle_kb();

      for(;;)//for(int i=0;i<3;i++)
      {
        servo_set_setpoint(1000);
        delay(100);
        servo_set_setpoint(1000);
        delay(100);
      }

      struct servo_log_entry *log;
      int log_samples;

      servo_stop_logging(&log, &log_samples);

      pp_printf("@respstart %d\n\r", log_samples);


      for(int i = 0; i < log_samples; i++)
      {
        pp_printf("@respdata %d %d %d %d\n\r", i, log[i].setpoint, log[i].error, log[i].y);
      }

for(;;)
{
        //pp_printf("last_x %d\n\r",last_x);
        //handle_kb();
      }

      servo_set_setpoint(2000);



    }


#endif

}


