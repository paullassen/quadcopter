#include <stdio.h>
#include <signal.h>
#include <math.h>
#include <robotcontrol.h>

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
// Global Variables
static int running = 0;
static rc_mpu_data_t data;
static rc_bmp_data_t bmp_data;

static int counter = 0;
static int  rax[BUF_LEN] = {0};
static int  ray[BUF_LEN] = {0};
static int  raz[BUF_LEN] = {0};
static int  rgx[BUF_LEN] = {0};
static int  rgy[BUF_LEN] = {0};
static int  rgz[BUF_LEN] = {0};
static double ax[BUF_LEN] = {0};
static double ay[BUF_LEN] = {0};
static double az[BUF_LEN] = {0};
static double gx[BUF_LEN] = {0};
static double gy[BUF_LEN] = {0};
static double gz[BUF_LEN] = {0};
static double alt[BMP_BUF_LEN] = {0};
static int   dt_loop[BUF_LEN] = {0};
static int   dt_collect[BUF_LEN] = {0};
// local functions
static void __collect_data(void);
static void __print_statistics(void);
uint64_t loop_start;
uint64_t collect_start;
/**
 * This is the IMU interrupt function.
 */
static void __collect_data(void)
{
	if( counter >= BUF_LEN ) return;

	collect_start = rc_nanos_since_epoch();

	if( counter%MPU_PER_BMP==0 )
	{
		rc_bmp_read(&bmp_data);
		alt[counter/MPU_PER_BMP] = bmp_data.alt_m;
	}

	rax[counter] = data.raw_accel[0];
	ray[counter] = data.raw_accel[1];
	raz[counter] = data.raw_accel[2];
	rgx[counter] = data.raw_gyro[0];
	rgy[counter] = data.raw_gyro[1];
	rgz[counter] = data.raw_gyro[2];
	ax[counter] = data.accel[0];
	ay[counter] = data.accel[1];
	az[counter] = data.accel[2];
	gx[counter] = data.gyro[0];
	gy[counter] = data.gyro[1];
	gz[counter] = data.gyro[2];
	
	uint64_t time_now = rc_nanos_since_epoch();
	dt_loop[counter] = counter == 0 ? 0 : time_now - loop_start;
	loop_start = time_now;
	dt_collect[counter] = time_now - collect_start;

	if( ++counter >= BUF_LEN ) running = 0;
	return;
}

static void __main_loop(void)
{
  counter = counter%MPU_PER_BMP;

  if( counter == 0 )
  {
    rc_bmp_read(&bmp_data);
  }
  // Collect Measurements
  //    Altitude
  //    Attitude
  
  // Collect Targets

  // Calculate Output
  
  // Update Output
  ++counter;
}

static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}

int main()
{
	//MPU config
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
	conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
	conf.dmp_fetch_accel_gyro=1;
	conf.dmp_sample_rate = SAMPLE_RATE;
	conf.dmp_interrupt_sched_policy = SCHED_FIFO;
	conf.dmp_interrupt_priority = 59;
	
	if( rc_bmp_init(16, BMP_FILTER_OFF) ) return -1;
	if(rc_bmp_read(&bmp_data)) return -1;

	if( rc_kill_existing_process(2.0) <= -2 ) return -1;
	rc_make_pid_file();
	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running = 1;

	// now set up the imu for dmp interrupt operation
	if(rc_mpu_initialize_dmp(&data, conf)){
		printf("rc_mpu_initialize_failed\n");
		return -1;
	}
	// write labels for what data will be printed and associate the interrupt
	// function to print data immediately after the header.
	rc_mpu_set_dmp_callback(&__collect_data);
	//now just wait, print_data() will be called by the interrupt
	while(running)	rc_usleep(100000);
	// shut things down
	rc_mpu_power_off();
	rc_bmp_power_off();
	rc_remove_pid_file();	
	printf("\n");
	fflush(stdout);
	return 0;
}


