#ifndef _QUADCOPTER_FLIGHT_C_
#define _QUADCOPTER_FLIGHT_C_

#include <rc/bmp.h>
#include <rc/mpu.h>

#include "state.h"

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21

#define SAMPLE_RATE 200
#define COLLECTION_TIME 500
#define BUF_LEN SAMPLE_RATE *COLLECTION_TIME

#define BMP_SAMPLE_RATE 25
#define MPU_PER_BMP 8
#define BMP_BUF_LEN BUF_LEN / MPU_PER_BMP

static void __main_loop(void);

typedef struct {
  drone_state_t roll_state[1];
  drone_state_t pitch_state[1];
  drone_state_t yaw_state[1];
  drone_state_t alt_state[1];

  rc_mpu_data_t *mpu_data;
  rc_bmp_data_t *bmp_data;

  double roll_pid[3];
  double pitch_pid[3];
  double yaw_pid[3];
  double alt_pid[3];

  int counter;
} attitude_controller_t;

void attitude_controller_init(attitude_controller_t *ac, rc_mpu_data_t *md,
                              rc_bmp_data_t *bd);

void attitude_controller_set_roll_pid(attitude_controller_t *ac, double p,
                                      double i, double d);
void attitude_controller_set_pitch_pid(attitude_controller_t *ac, double p,
                                       double i, double d);
void attitude_controller_set_yaw_pid(attitude_controller_t *ac, double p,
                                     double i, double d);
void attitude_controller_set_alt_pid(attitude_controller_t *ac, double p,
                                     double i, double d);

void attitude_controller_update(attitude_controller_t *ac);

void attitude_controller_set_roll_target(attitude_controller_t *ac, double t);
void attitude_controller_set_pitch_target(attitude_controller_t *ac, double t);
void attitude_controller_set_yaw_target(attitude_controller_t *ac, double t);
void attitude_controller_set_alt_target(attitude_controller_t *ac, double t);

void attitude_controller_set_targets(attitude_controller_t *ac, double *targs);

double attitude_controller_get_roll_target(attitude_controller_t *ac);
double attitude_controller_get_pitch_target(attitude_controller_t *ac);
double attitude_controller_get_yaw_target(attitude_controller_t *ac);
double attitude_controller_get_alt_target(attitude_controller_t *ac);

double attitude_controller_get_roll_error(attitude_controller_t *ac);
double attitude_controller_get_pitch_error(attitude_controller_t *ac);
double attitude_controller_get_yaw_error(attitude_controller_t *ac);
double attitude_controller_get_alt_error(attitude_controller_t *ac);

double attitude_controller_get_roll_state(attitude_controller_t *ac);
double attitude_controller_get_pitch_state(attitude_controller_t *ac);
double attitude_controller_get_yaw_state(attitude_controller_t *ac);
double attitude_controller_get_alt_state(attitude_controller_t *ac);

double attitude_controller_get_roll_state_derivative(attitude_controller_t *ac);
double
attitude_controller_get_pitch_state_derivative(attitude_controller_t *ac);
double attitude_controller_get_yaw_state_derivative(attitude_controller_t *ac);
double attitude_controller_get_alt_state_derivative(attitude_controller_t *ac);
#endif
