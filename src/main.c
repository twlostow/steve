#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"

#include "pp-printf.h"
#include "biquad.h"
#include "usart.h"
#include "servo.h"
#include "rpc.h"
#include "motors.h"
#include "common.h"

void fet_charge(int on);
void fet_break(int on);
int is_touchdown();
void hv_init();


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
  int dvdt = rpc_pop_int32(rq);
  struct servo_log_entry *log;
  int log_samples;

  servo_set_setpoint(init_setpoint);
  servo_start_logging( 0 );
  delay(200);
  int i;
  for(i=0;i<1;i++)
  {
  servo_set_target(target_setpoint, dvdt);
//  while(!servo_position_ready());
  delay(20);
  servo_set_target(init_setpoint, dvdt);
  //while(!servo_position_ready());
  delay(20);
  }
  delay(200);
  servo_stop_logging(&log, &log_samples);

//  pp_printf("log has %d samples\n\r", log_samples);
  rpc_answer_send(RPC_ID_SERVO_RESPONSE, log_samples * sizeof(struct servo_log_entry), log);
}


void cmd_profile_height(struct rpc_request *rq)
{
  int initial_setpoint = rpc_pop_int32(rq);
  int n_points = rpc_pop_int32(rq);
  pp_printf("initial_setp %d np %d\n", initial_setpoint, n_points);
  servo_set_target(initial_setpoint, 1);
  while(!servo_position_ready());
  int i;
  delay(10);
  int p, r;



  for(i=0;i<n_points;i++)
  {
    esc_control_update();
    servo_set_target(300, 15);
    while(!is_touchdown());// pp_printf("target %d setp %d sen %d\n", servo_get_target(), servo_get_setpoint(), servo_get_sensor());


    p = servo_get_sensor();
    r = esc_get_radial_position();

    servo_set_setpoint(p + 300);
    while(is_touchdown());

//    delay(5);
    rpc_answer_push(&p, 4);
    rpc_answer_push(&r, 4);
    //delay(32);
    //pp_printf("touchdown at %d %d %d LO %d\n\r", p, servo_get_sensor(), servo_get_setpoint(), esc_get_radial_position());
  }

  rpc_answer_commit(RPC_ID_PROFILE_HEIGHT);
  servo_set_target(initial_setpoint, 1);

}

void cmd_ldrive_step(struct rpc_request *rq)
{
  int count = rpc_pop_int32(rq);

  ldrive_advance_by(count, 10);
}

void cmd_ldrive_read_encoder(struct rpc_request *rq)
{
  int i = ldrive_get_encoder_value(0);
  int q = ldrive_get_encoder_value(1);


  rpc_answer_push(&i, 4);
  rpc_answer_push(&q, 4);

  rpc_answer_commit(RPC_ID_LDRIVE_READ_ENCODER);
}

void cmd_ldrive_go_home()
{
  ldrive_go_home();
}

void cmd_ldrive_check_idle()
{
  int idle = ldrive_idle();
  rpc_answer_push(&idle, 4);
  rpc_answer_commit(RPC_ID_LDRIVE_CHECK_IDLE);
}

void main_loop()
{
  struct rpc_request rq;
  struct timeout keepalive;

  tmo_init(&keepalive, 1000);
  while (1)
  {
    esc_control_update();
    ldrive_update();

    //pp_printf("touch %d\n\r", is_touchdown());
    if(tmo_hit(&keepalive))
    {
      pp_printf("Heartbeat [enc_i = %d, enc_q = %d]!\n\r", ldrive_get_encoder_value(0),  ldrive_get_encoder_value(1));
      //pp_printf("servo %d %d\n\r", servo_get_sensor(), servo_get_setpoint());
    }

    if(!rpc_request_get(&rq))
      continue;


    switch(rq.id)
    {
      case RPC_ID_ADC_TEST: cmd_adc_test(&rq); break;
      case RPC_ID_GET_ESC_SPEED: cmd_esc_get_speed(&rq); break;
      case RPC_ID_SET_ESC_SPEED: cmd_esc_set_speed(&rq); break;
      case RPC_ID_SERVO_RESPONSE: cmd_servo_response(&rq); break;
      case RPC_ID_PROFILE_HEIGHT: cmd_profile_height(&rq); break;
      case RPC_ID_LDRIVE_STEP: cmd_ldrive_step(&rq); break;
      case RPC_ID_LDRIVE_READ_ENCODER: cmd_ldrive_read_encoder(&rq); break;
      case RPC_ID_LDRIVE_GO_HOME: cmd_ldrive_go_home(&rq); break;
      case RPC_ID_LDRIVE_CHECK_IDLE: cmd_ldrive_check_idle(&rq); break;

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
    ldrive_init();
    hv_init();
    esc_init();
    esc_set_speed(10.0);

    strobe_led(0);
    main_loop();

    ldrive_go_home();

    while(!ldrive_idle())
      ldrive_update();


    ldrive_advance_by(-500, 4);

    for(;;)
          ldrive_update();


    return 0;
}


