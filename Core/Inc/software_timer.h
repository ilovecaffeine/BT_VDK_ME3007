/*
 * software_timer.h
 *
 *  Created on: Sep 17, 2024
 *      Author: admin
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

extern int timer0_flag;
extern int timer0_counter;
extern int timer1_flag;
extern int timer1_counter;
extern int timer2_flag;
extern int timer2_counter;
extern int timer3_flag;
extern int timer3_counter;
extern int timer4_flag;
extern int timer4_counter;
extern int timer5_flag;
extern int timer5_counter;
extern int TIMER_CYCLE;

void setTimer0(int duration);
void setTimer1(int duration);
void setTimer2(int duration);
void setTimer3(int duration);
void setTimer4(int duration);
void setTimer5(int duration);
void timer_run();


#endif /* INC_SOFTWARE_TIMER_H_ */
