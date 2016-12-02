#include <stdio.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <iostream>

#include "lcmt_motor_command.h"
#include "lcmt_acrobot_u.h"


using namespace std;

lcm_t * lcm = NULL;

lcmt_acrobot_u newest_command;
lcmt_acrobot_u previous_command;
lcmt_motor_command motor_command;
long int last_time_ms;

/*
Go through all the command signals and only publish to the motor controller when the command for its motor has changed
*/
void lcm_publish_control(void) 
{
  lcmt_motor_command_publish(lcm,"acrobot_in", &motor_command);
	memcpy(&previous_command,&newest_command,sizeof(lcmt_acrobot_u));
}

/*
Initialize everything so we don't send random bits of memory to the motor that controls the killsaw
*/
void Initialize(void){
	newest_command.tau = 0;		//Elbow Torque
	newest_command.timestamp = 0;

	motor_command.timestamp = 0;
	motor_command.command = 0;
	motor_command.command_type = 4;         // current
}

/*
Pull the relevant bits of data out of the control message and stick them in the controller specific messages
*/
void translate_control(void)
{
	motor_command.command = newest_command.tau;

	motor_command.timestamp = newest_command.timestamp;
}

void lcm_command_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_acrobot_u *msg, void *user)
{
	memcpy(&newest_command,msg,sizeof(lcmt_acrobot_u));
	translate_control();
	lcm_publish_control();

  struct timeval time;
  gettimeofday(&time, NULL);
  last_time_ms = time.tv_sec*1000 + time.tv_usec/1000;
}

int main(int argc, char** argv)
{
	//Set up LCM
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=0");
	if (!lcm) return 1;
	// subscribe to relevant channels:
	lcmt_acrobot_u_subscription_t * command_sub = lcmt_acrobot_u_subscribe (lcm, "acrobot_u", &lcm_command_handler, NULL);
	Initialize();
  
  int timeout_ms = 500;
  struct timeval time;
  gettimeofday(&time, NULL);
  last_time_ms = time.tv_sec*1000 + time.tv_usec/1000;
  long int time_ms;
	while (1)
	{
    gettimeofday(&time, NULL);
    time_ms = time.tv_sec*1000 + time.tv_usec/1000;
    if (time_ms-last_time_ms > timeout_ms) {
      // timeout! publish a zero control message
      last_time_ms = time_ms;
      newest_command.tau = 0;
      newest_command.timestamp = time_ms;
      lcm_publish_control();
    }
		lcm_handle_timeout(lcm,50);
	}

	lcmt_acrobot_u_unsubscribe (lcm, command_sub);
	lcm_destroy (lcm);
	return 0;
}
