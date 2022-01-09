#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <rc/bmp.h>
#include <rc/mpu.h>
#include <rc/start_stop.h>
#include <rc/adc.h>
#include <rc/math/vector.h>

#include "attitude_controller.h"
#include "joystick_server.h"
#include "motor.h"
#include "state.h"

static joystick_t js[1];
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
static attitude_controller_t att_ctrl[1];
static motor_interface_t m;
static int loop_counter;

int sensor_init(rc_mpu_data_t *md, rc_bmp_data_t *bd) {
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.i2c_bus = I2C_BUS;
  conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
  conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
  conf.dmp_fetch_accel_gyro = 1;
  conf.dmp_sample_rate = SAMPLE_RATE;
  conf.dmp_interrupt_sched_policy = SCHED_FIFO;
  conf.dmp_interrupt_priority = 59;
  conf.enable_magnetometer = 1;

  if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_OFF))
    return -1;
  if (rc_bmp_read(bd))
    return -1;

  if (rc_kill_existing_process(2.0) <= -2)
    return -1;
  rc_make_pid_file();

  if (rc_mpu_initialize_dmp(md, conf)) {
    printf("rc_mpu_initialize_failed\n");
    return -1;
  }
}

void __main_loop(void) {
  if( loop_counter < 200 ){
    attitude_controller_update(att_ctrl);
    loop_counter++;
    return;
  } else if(loop_counter == 200){
    attitude_controller_set_yaw_target(att_ctrl, attitude_controller_get_yaw_state(att_ctrl));
    attitude_controller_set_alt_target(att_ctrl, attitude_controller_get_alt_state(att_ctrl));
    loop_counter++;
  }

  double a_targ, r_targ, p_targ, y_targ;
  r_targ = joystick_get_norm(js, JS_R_STICK_LR)*4*M_PI/180;
  p_targ = joystick_get_norm(js, JS_R_STICK_UD)*4*M_PI/180;

  y_targ = joystick_get_norm(js, JS_L_STICK_LR) * 4 * M_PI / 180 / SAMPLE_RATE;
  y_targ += attitude_controller_get_yaw_target(att_ctrl);
  if (y_targ > M_PI) {
    y_targ = y_targ - 2 * M_PI;
  } else if (y_targ < -M_PI) {
    y_targ = 2 * M_PI + y_targ;
  }

  a_targ = (joystick_get_norm(js, JS_R_BUMPER) -
            joystick_get_norm(js, JS_L_BUMPER)) /
           SAMPLE_RATE;
  a_targ += attitude_controller_get_alt_target(att_ctrl);

  attitude_controller_set_yaw_target(att_ctrl, y_targ);
  attitude_controller_set_pitch_target(att_ctrl, p_targ);
  attitude_controller_set_roll_target(att_ctrl, r_targ);
  attitude_controller_set_alt_target(att_ctrl, a_targ);

  attitude_controller_update(att_ctrl);
  
  rc_vector_t * wrench =  attitude_controller_calculate_wrench(att_ctrl);
  //write_motor_wrench(&m, attitude_controller_calculate_wrench(att_ctrl));

}

int main() {
  loop_counter = 0;

  joystick_init(js);
  sensor_init(&mpu_data, &bmp_data);
  attitude_controller_init(att_ctrl, &mpu_data, &bmp_data);
  attitude_controller_set_alt_pid(att_ctrl, 0.4, 0.0, 10);
  init_motor_interface(&m);

  int running = 1;
  double mtr[4] = {0};
  printf("\nStarting Joystick handler\n");
  joystick_start(js);
  usleep(10000);
  printf("\nSetting dmp_callback\n");

  rc_adc_init();
  rc_mpu_set_dmp_callback(&__main_loop);
  while (running) {
    if (joystick_get_raw(js, JS_START_BUTTON)) // End on start_button press
      running = 0;

    for (int j = 0; j < 4; ++j) {
      if (joystick_get_raw(js, j)) {
        mtr[j] = joystick_get_norm(js, JS_R_BUMPER);
      } else {
        mtr[j] = 0;
      }
    }
    printf("\rName    :  Roll    Pitch  Yaw    alt   \n");
    printf("\rTarget  :  %+5.3f   %+5.3f   %+5.3f   %+5.3f   \n",
           attitude_controller_get_roll_target(att_ctrl) * 180 / M_PI,
           attitude_controller_get_pitch_target(att_ctrl) * 180 / M_PI,
           attitude_controller_get_yaw_target(att_ctrl) * 180 / M_PI,
           attitude_controller_get_alt_target(att_ctrl));
    printf("\rState   :  %+5.3f   %+5.3f   %+5.3f   %+5.3f   \n",
           attitude_controller_get_roll_state(att_ctrl) * 180 / M_PI,
           attitude_controller_get_pitch_state(att_ctrl) * 180 / M_PI,
           attitude_controller_get_yaw_state(att_ctrl) * 180 / M_PI,
           attitude_controller_get_alt_state(att_ctrl));
    printf("\rError   :  %+5.3f   %+5.3f   %+5.3f   %+5.3f   \n",
           attitude_controller_get_roll_error(att_ctrl) * 180 / M_PI,
           attitude_controller_get_pitch_error(att_ctrl) * 180 / M_PI,
           attitude_controller_get_yaw_error(att_ctrl) * 180 / M_PI,
           attitude_controller_get_alt_error(att_ctrl));
    printf("\rError_d :  %+5.3f   %+5.3f   %+5.3f   %+5.3f   \n",
           attitude_controller_get_roll_error_derivative(att_ctrl) * 180 / M_PI,
           attitude_controller_get_pitch_error_derivative(att_ctrl) * 180 / M_PI,
           attitude_controller_get_yaw_error_derivative(att_ctrl) * 180 / M_PI,
           attitude_controller_get_alt_error_derivative(att_ctrl));
    printf("\rError_i :  %+5.3f   %+5.3f   %+5.3f   %+5.3f   \n",
           attitude_controller_get_roll_error_integral(att_ctrl) * 180 / M_PI,
           attitude_controller_get_pitch_error_integral(att_ctrl) * 180 / M_PI,
           attitude_controller_get_yaw_error_integral(att_ctrl) * 180 / M_PI,
           attitude_controller_get_alt_error_integral(att_ctrl));
    printf("\rWrench :  % 5f   % 5f   % 5f   % 5f   \n",
           att_ctrl->wrench.d[1],
           att_ctrl->wrench.d[2],
           att_ctrl->wrench.d[3],
           att_ctrl->wrench.d[0]);
//    printf("\rMtrVol: %5d    %5d    %5d     %5d   \n",
//            m.pulse_width[0],
//            m.pulse_width[1],
//            m.pulse_width[2],
//            m.pulse_width[3]);
    printf("\rBattery:  % 5f\033[7A",
         rc_adc_dc_jack());
    usleep(10000);
  }

  rc_mpu_power_off();
  rc_bmp_power_off();
  rc_remove_pid_file();

  for (int i = 0; i < 4; ++i)
    mtr[i] = 0;
  write_motor_pct(&m, mtr);
  destroy_motor_interface(&m);
  running = 0;
  joystick_stop(js);
  printf("\n\n\n\n\n\n\n\n\n\n\n\nFinished Reading\n");
  joystick_print(js);
  printf("\n");
  rc_adc_cleanup();
  return 0;
}
