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

static void __print_statistics(void){
	double rax_mean = 0;
	double ray_mean = 0;
	double raz_mean = 0;
	double rgx_mean = 0;
	double rgy_mean = 0;
	double rgz_mean = 0;
	double  ax_mean = 0;
	double  ay_mean = 0;
	double  az_mean = 0;
	double  gx_mean = 0;
	double  gy_mean = 0;
	double  gz_mean = 0;
	double  dt_loop_mean = 0;
	double  dt_collect_mean = 0;
	double alt_mean = 0;

	double rax_min = 1e6;
	double ray_min = 1e6;
	double raz_min = 1e6;
	double rgx_min = 1e6;
	double rgy_min = 1e6;
	double rgz_min = 1e6;
	double  ax_min = 1e6;
	double  ay_min = 1e6;
	double  az_min = 1e6;
	double  gx_min = 1e6;
	double  gy_min = 1e6;
	double  gz_min = 1e6;
	double  dt_loop_min = 1e9;
	double  dt_collect_min = 1e6;
	double alt_min = 1e6;

	double rax_max = -1e6;
	double ray_max = -1e6;
	double raz_max = -1e6;
	double rgx_max = -1e6;
	double rgy_max = -1e6;
	double rgz_max = -1e6;
	double  ax_max = -1e6;
	double  ay_max = -1e6;
	double  az_max = -1e6;
	double  gx_max = -1e6;
	double  gy_max = -1e6;
	double  gz_max = -1e6;
	double  dt_loop_max = -1e6;
	double  dt_collect_max = -1e6;
	double alt_max= -1e6;

	double rax_std = 0;
	double ray_std = 0;
	double raz_std = 0;
	double rgx_std = 0;
	double rgy_std = 0;
	double rgz_std = 0;
	double  ax_std = 0;
	double  ay_std = 0;
	double  az_std = 0;
	double  gx_std = 0;
	double  gy_std = 0;
	double  gz_std = 0;
	double  dt_loop_std = 0;
	double  dt_collect_std = 0;
	double alt_std = 0;
	int i = 0;
	for( i = 0; i < counter; ++i ){
		rax_mean +=  rax[i];
		ray_mean +=  ray[i];
                raz_mean +=  raz[i];
                rgx_mean +=  rgx[i];
                rgy_mean +=  rgy[i];
                rgz_mean +=  rgz[i];
                 ax_mean +=   ax[i];
                 ay_mean +=   ay[i];
                 az_mean +=   az[i];
                 gx_mean +=   gx[i];
                 gy_mean +=   gy[i];
                 gz_mean +=   gz[i];
                 dt_loop_mean +=   dt_loop[i];
                 dt_collect_mean +=   dt_collect[i];

		 if( rax[i] < rax_min ) rax_min = rax[i];
		 if( ray[i] < ray_min ) ray_min = ray[i];
		 if( raz[i] < raz_min ) raz_min = raz[i];
		 if( rgx[i] < rgx_min ) rgx_min = rgx[i];
		 if( rgy[i] < rgy_min ) rgy_min = rgy[i];
		 if( rgz[i] < rgz_min ) rgz_min = rgz[i];
		 if(  ax[i] <  ax_min )  ax_min =  ax[i];
		 if(  ay[i] <  ay_min )  ay_min =  ay[i];
		 if(  az[i] <  az_min )  az_min =  az[i];
		 if(  gx[i] <  gx_min )  gx_min =  gx[i];
		 if(  gy[i] <  gy_min )  gy_min =  gy[i];
		 if(  gz[i] <  gz_min )  gz_min =  gz[i];
		 if(  dt_loop[i] <  dt_loop_min )  dt_loop_min = i==0 ? dt_loop_min : dt_loop[i];
		 if(  dt_collect[i] <  dt_collect_min )  dt_collect_min =  dt_collect[i];

		 if( rax[i] > rax_max ) rax_max = rax[i];
		 if( ray[i] > ray_max ) ray_max = ray[i];
		 if( raz[i] > raz_max ) raz_max = raz[i];
		 if( rgx[i] > rgx_max ) rgx_max = rgx[i];
		 if( rgy[i] > rgy_max ) rgy_max = rgy[i];
		 if( rgz[i] > rgz_max ) rgz_max = rgz[i];
		 if(  ax[i] >  ax_max )  ax_max =  ax[i];
		 if(  ay[i] >  ay_max )  ay_max =  ay[i];
		 if(  az[i] >  az_max )  az_max =  az[i];
		 if(  gx[i] >  gx_max )  gx_max =  gx[i];
		 if(  gy[i] >  gy_max )  gy_max =  gy[i];
		 if(  gz[i] >  gz_max )  gz_max =  gz[i];
		 if(  dt_loop[i] >  dt_loop_max )  dt_loop_max =  i==0 ? dt_loop_max : dt_loop[i];
		 if(  dt_collect[i] >  dt_collect_max )  dt_collect_max =  dt_collect[i];

		 if( i%MPU_PER_BMP==0 ){
			alt_mean += alt[i/MPU_PER_BMP];
			if( alt[i/MPU_PER_BMP] < alt_min ) alt_min = alt[i/MPU_PER_BMP];
			if( alt[i/MPU_PER_BMP] > alt_max ) alt_max = alt[i/MPU_PER_BMP];
		 }
	}

	rax_mean /= counter;
	ray_mean /= counter;
        raz_mean /= counter;
        rgx_mean /= counter;
        rgy_mean /= counter;
        rgz_mean /= counter;
         ax_mean /= counter;
         ay_mean /= counter;
         az_mean /= counter;
         gx_mean /= counter;
         gy_mean /= counter;
         gz_mean /= counter;
         dt_loop_mean /= (counter-1);
         dt_collect_mean /= counter;
	 alt_mean /= counter/MPU_PER_BMP;

	for( i = 0; i < counter; ++i ){
		rax_std +=  (rax[i] - rax_mean) * (rax[i] - rax_mean);
		ray_std +=  (ray[i] - ray_mean) * (ray[i] - ray_mean);
                raz_std +=  (raz[i] - raz_mean) * (raz[i] - raz_mean);
                rgx_std +=  (rgx[i] - rgx_mean) * (rgx[i] - rgx_mean);
                rgy_std +=  (rgy[i] - rgy_mean) * (rgy[i] - rgy_mean);
                rgz_std +=  (rgz[i] - rgz_mean) * (rgz[i] - rgz_mean);
                 ax_std +=  ( ax[i] -  ax_mean) * ( ax[i] -  ax_mean);
                 ay_std +=  ( ay[i] -  ay_mean) * ( ay[i] -  ay_mean);
                 az_std +=  ( az[i] -  az_mean) * ( az[i] -  az_mean);
                 gx_std +=  ( gx[i] -  gx_mean) * ( gx[i] -  gx_mean);
                 gy_std +=  ( gy[i] -  gy_mean) * ( gy[i] -  gy_mean);
                 gz_std +=  ( gz[i] -  gz_mean) * ( gz[i] -  gz_mean);
                 dt_loop_std +=  i==0 ? 0 : ( dt_loop[i] -  dt_loop_mean)*1e-6 * ( dt_loop[i] -  dt_loop_mean)*1e-6;
                 dt_collect_std +=  ( dt_collect[i] -  dt_collect_mean)*1e-6 * ( dt_collect[i] -  dt_collect_mean)*1e-6;
		 if( i%MPU_PER_BMP==0 ) alt_std += (alt[i/MPU_PER_BMP] - alt_mean) * (alt[i/MPU_PER_BMP] - alt_mean);
	}

	rax_std /= counter;
	ray_std /= counter;
        raz_std /= counter;
        rgx_std /= counter;
        rgy_std /= counter;
        rgz_std /= counter;
         ax_std /= counter;
         ay_std /= counter;
         az_std /= counter;
         gx_std /= counter;
         gy_std /= counter;
         gz_std /= counter;
         dt_loop_std /= counter-1;
         dt_collect_std /= counter;
	alt_std /= counter/MPU_PER_BMP;
	printf("\n%d samples collected at %dhz for %.3fs\n", counter, SAMPLE_RATE, counter/(double)SAMPLE_RATE);
	printf("\t Raw_Accel XYZ \t Raw_Gyro XYZ \t Accel XYZ \t Gyro XYZ\n");
	printf("Mean:\n");
	printf("    x\t%f\t%f\t%f\t%f\n", rax_mean, rgx_mean, ax_mean, gx_mean);
	printf("    y\t%f\t%f\t%f\t%f\n", ray_mean, rgy_mean, ay_mean, gy_mean);
	printf("    z\t%f\t%f\t%f\t%f\n", raz_mean, rgz_mean, az_mean, gz_mean);

	printf("Min:\n");
	printf("    x\t%f\t%f\t%f\t%f\n", rax_min-rax_mean, rgx_min-rgx_mean, ax_min-ax_mean, gx_min-gx_mean);
	printf("    y\t%f\t%f\t%f\t%f\n", ray_min-ray_mean, rgy_min-rgy_mean, ay_min-ay_mean, gy_min-gy_mean);
	printf("    z\t%f\t%f\t%f\t%f\n", raz_min-raz_mean, rgz_min-rgz_mean, az_min-az_mean, gz_min-gz_mean);
	
	printf("Max:\n");
	printf("    x\t%f\t%f\t%f\t%f\n", rax_max-rax_mean, rgx_max-rgx_mean, ax_max-ax_mean, gx_max-gx_mean);
	printf("    y\t%f\t%f\t%f\t%f\n", ray_max-ray_mean, rgy_max-rgy_mean, ay_max-ay_mean, gy_max-gy_mean);
	printf("    z\t%f\t%f\t%f\t%f\n", raz_max-raz_mean, rgz_max-rgz_mean, az_max-az_mean, gz_max-gz_mean);
	
	printf("Std:\n");
	printf("    x\t%f\t%f\t%f\t%f\n", sqrt(rax_std), sqrt(rgx_std), sqrt(ax_std), sqrt(gx_std));
	printf("    y\t%f\t%f\t%f\t%f\n", sqrt(ray_std), sqrt(rgy_std), sqrt(ay_std), sqrt(gy_std));
	printf("    z\t%f\t%f\t%f\t%f\n", sqrt(raz_std), sqrt(rgz_std), sqrt(az_std), sqrt(gz_std));
	
	printf("\nAltitude\n");
	printf("Mean:%f Min:%f Max:%f Std:%f\n",alt_mean, alt_min-alt_mean, alt_max-alt_mean, alt_std);

	printf("\n Sample Time\n");
	printf("Loop    Mean:\t%f\tStd:\t%f\n", dt_loop_mean*1e-6, sqrt(dt_loop_std));
	printf("Collect Mean:\t%f\tStd:\t%f\n", dt_collect_mean*1e-6, sqrt(dt_collect_std));
	printf("Loop    Min:\t%f\tMax:\t%f\n", dt_loop_min*1e-6-dt_loop_mean*1e-6, dt_loop_max*1e-6-dt_loop_mean*1e-6);
	printf("Collect Min:\t%f\tMax:\t%f\n", dt_collect_min*1e-6-dt_collect_mean*1e-6, dt_collect_max*1e-6-dt_collect_mean*1e-6);

	printf("\n");
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
	__print_statistics();
	// shut things down
	rc_mpu_power_off();
	rc_bmp_power_off();
	rc_remove_pid_file();	
	printf("\n");
	fflush(stdout);
	return 0;
}


