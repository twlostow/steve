#ifndef __MOTORS_H
#define __MOTORS_H

void esc_init();
void esc_enable_power(int enable);
float esc_get_speed_rps();
void esc_set_speed ( float setpoint_rps );
void esc_control_update();
int esc_get_radial_position();

void ldrive_init();
void ldrive_step( int dir );

#endif
