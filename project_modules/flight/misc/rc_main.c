#include <pthread.h>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <robotcontrol.h>

#include "state.h"
#include "joystick_server.h"
// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

#define SAMPLE_RATE 200
#define COLLECTION_TIME 500
#define BUF_LEN SAMPLE_RATE*COLLECTION_TIME

#define BMP_SAMPLE_RATE 25
#define MPU_PER_BMP 8
#define BMP_BUF_LEN BUF_LEN/MPU_PER_BMP

static void __main_loop(void);
// Global Variables
double rc_ringbuf_mean(rc_ringbuf_t * buf)
{
	int i;
	double mean;
	// calculate mean
	mean = 0.0f;
	for(i=0;i<buf->size;i++) mean+=buf->d[i];

	return mean/(double)buf->size;
}

rc_mpu_data_t mpu_data;

drone_state_t roll_state[1];
drone_state_t pitch_state[1];
drone_state_t yaw_state[1];
drone_state_t alt_state[1];

joystick_t joystick[1];

typedef struct {
  int running;
  int counter;
  rc_bmp_data_t bmp_data;
  
  rc_ringbuf_t * fz;
  rc_ringbuf_t * tx;
  rc_ringbuf_t * ty;
  rc_ringbuf_t * tz;

} global_vars_t;

global_vars_t glob_var;

static void __main_loop(void)
{

  global_vars_t * gv = &glob_var;
  gv->counter = gv->counter%MPU_PER_BMP;

  drone_state_set_target(yaw_state, y_targ);
  drone_state_set_target(pitch_state, p_targ);
  drone_state_set_target(roll_state, r_targ);
  drone_state_set_target(alt_state, a_targ);

  if( gv->counter == 0 )
  {
    rc_bmp_read(&gv->bmp_data);
    drone_state_update(alt_state, gv->bmp_data.alt_m);
  }
  

  drone_state_update(yaw_state, mpu_data.fused_TaitBryan[TB_YAW_Z]*180/M_PI);
  drone_state_update(roll_state, mpu_data.fused_TaitBryan[TB_ROLL_Y]*180/M_PI);
  drone_state_update(pitch_state, mpu_data.fused_TaitBryan[TB_PITCH_X]*-180/M_PI);
  printf("\rnames:  yaw    pitch   roll   alt \n");

  printf("\rstate: %0.3f  %0.3f  %0.3f  %0.3f\n", 
            rc_ringbuf_get_value(yaw_state->state,0),
            rc_ringbuf_get_value(pitch_state->state,0),
            rc_ringbuf_get_value(roll_state->state,0),
            rc_ringbuf_get_value(alt_state->state,0));
  printf("\rerror: %0.3f  %0.3f  %0.3f  %0.3f\n", 
            rc_ringbuf_get_value(yaw_state->error,0),
            rc_ringbuf_get_value(pitch_state->error,0),
            rc_ringbuf_get_value(roll_state->error,0),
            rc_ringbuf_get_value(alt_state->error,0));
  printf("\rderiv: %0.3f  %0.3f  %0.3f  %0.3f\033[3A", 
            rc_ringbuf_get_value(yaw_state->state_derivative,0),
            rc_ringbuf_get_value(pitch_state->state_derivative,0),
            rc_ringbuf_get_value(roll_state->state_derivative,0),
            rc_ringbuf_get_value(alt_state->state_derivative,0));
  
  // Collect Targets

  // Calculate Output
  
  // Update Output
  ++gv->counter;
}


static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	glob_var.running=0;
	return;
}

int main()
{
  global_vars_t * gv = &glob_var;

	//MPU config
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	conf.dmp_fetch_accel_gyro=1;
	conf.dmp_sample_rate = SAMPLE_RATE;
	conf.dmp_interrupt_sched_policy = SCHED_FIFO;
	conf.dmp_interrupt_priority = 59;
  conf.enable_magnetometer = 1;
	
	if( rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_OFF) ) return -1;
	if(rc_bmp_read(&gv->bmp_data)) return -1;

	if( rc_kill_existing_process(2.0) <= -2 ) return -1;
	rc_make_pid_file();
	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	gv->running = 1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&mpu_data, conf)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}

  drone_state_init(alt_state, BMP_SAMPLE_RATE, BMP_SAMPLE_RATE);
  drone_state_init(yaw_state, (int) SAMPLE_RATE/10, SAMPLE_RATE);
  drone_state_init(pitch_state, (int) SAMPLE_RATE/10, SAMPLE_RATE);
  drone_state_init(roll_state, (int) SAMPLE_RATE/10, SAMPLE_RATE);

	// write labels for what data will be printed and associate the interrupt
	// function to print data immediately after the header.
	rc_mpu_set_dmp_callback(&__main_loop);
	//now just wait, print_data() will be called by the interrupt
	while(gv->running)	rc_usleep(100000);
	// shut things down
	rc_mpu_power_off();
	rc_bmp_power_off();
	rc_remove_pid_file();	

  drone_state_destroy(alt_state);
  drone_state_destroy(yaw_state);
  drone_state_destroy(pitch_state);
  drone_state_destroy(roll_state);

	printf("\n\n\n\n");
	fflush(stdout);
	return 0;
}
