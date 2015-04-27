#ifndef __MOTORS_H
#define __MOTORS_H

void esc_init();
void esc_enable_power(int enable);
float esc_get_speed_rps();
void esc_set_speed ( float setpoint_rps );
void esc_control_update();

void head_init();
void head_step( int dir );

#endif
