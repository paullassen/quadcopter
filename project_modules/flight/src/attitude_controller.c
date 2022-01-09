#include <rc/bmp.h>
#include <rc/mpu.h>
#include <rc/math/vector.h>

#include "attitude_controller.h"
#include "state.h"

void attitude_controller_init(attitude_controller_t *ac, rc_mpu_data_t *md,
                              rc_bmp_data_t *bd) {
  ac->mpu_data = md;
  ac->bmp_data = bd;

  ac->counter = 0;

  for (int i = 0; i < 3; ++i) {
    ac->roll_pid[i] = 0;
    ac->pitch_pid[i] = 0;
    ac->yaw_pid[i] = 0;
    ac->alt_pid[i] = 0;
  }

  drone_state_init(ac->roll_state, SAMPLE_RATE / 10, SAMPLE_RATE);
  drone_state_init(ac->pitch_state, SAMPLE_RATE / 10, SAMPLE_RATE);
  drone_state_init(ac->yaw_state, SAMPLE_RATE / 10, SAMPLE_RATE);
  drone_state_init(ac->alt_state, BMP_SAMPLE_RATE / 5, BMP_SAMPLE_RATE);

  rc_vector_alloc(&ac->wrench,4);
}

void attitude_controller_set_roll_pid(attitude_controller_t *ac, double p,
                                      double i, double d) {
  ac->roll_pid[0] = p;
  ac->roll_pid[1] = i;
  ac->roll_pid[2] = d;
}
void attitude_controller_set_pitch_pid(attitude_controller_t *ac, double p,
                                       double i, double d) {
  ac->pitch_pid[0] = p;
  ac->pitch_pid[1] = i;
  ac->pitch_pid[2] = d;
}
void attitude_controller_set_yaw_pid(attitude_controller_t *ac, double p,
                                     double i, double d) {
  ac->yaw_pid[0] = p;
  ac->yaw_pid[1] = i;
  ac->yaw_pid[2] = d;
}
void attitude_controller_set_alt_pid(attitude_controller_t *ac, double p,
                                     double i, double d) {
  ac->alt_pid[0] = p;
  ac->alt_pid[1] = i;
  ac->alt_pid[2] = d;
}

void attitude_controller_update(attitude_controller_t *ac) {
  ac->counter %= MPU_PER_BMP;

  if (!ac->counter) {
    rc_bmp_read(ac->bmp_data);
    drone_state_update(ac->alt_state, ac->bmp_data->alt_m);
  }
  drone_state_update(ac->roll_state, ac->mpu_data->fused_TaitBryan[0]);
  drone_state_update(ac->pitch_state, ac->mpu_data->fused_TaitBryan[1]);
  drone_state_update(ac->yaw_state, ac->mpu_data->fused_TaitBryan[2]);
  ++ac->counter;
}

rc_vector_t * attitude_controller_calculate_wrench(attitude_controller_t *ac) {
  ac->wrench.d[0] = drone_state_get_error(ac->alt_state) * ac->alt_pid[0] +
         drone_state_get_error_integral(ac->alt_state) * ac->alt_pid[1] +
         drone_state_get_error_derivative(ac->alt_state) * ac->alt_pid[2];

  ac->wrench.d[1] = drone_state_get_error(ac->roll_state) * ac->roll_pid[0] +
         drone_state_get_error_integral(ac->roll_state) * ac->roll_pid[1] +
         drone_state_get_error_derivative(ac->roll_state) * ac->roll_pid[2];

  ac->wrench.d[2] = drone_state_get_error(ac->pitch_state) * ac->pitch_pid[0] +
         drone_state_get_error_integral(ac->pitch_state) * ac->pitch_pid[1] +
         drone_state_get_error_derivative(ac->pitch_state) * ac->pitch_pid[2];

  ac->wrench.d[3] = drone_state_get_error(ac->yaw_state) * ac->yaw_pid[0] +
         drone_state_get_error_integral(ac->yaw_state) * ac->yaw_pid[1] +
         drone_state_get_error_derivative(ac->yaw_state) * ac->yaw_pid[2];

  return &ac->wrench;
}

void attitude_controller_set_roll_target(attitude_controller_t *ac, double t) {
  drone_state_set_target(ac->roll_state, t);
}
void attitude_controller_set_pitch_target(attitude_controller_t *ac, double t) {
  drone_state_set_target(ac->pitch_state, t);
}
void attitude_controller_set_yaw_target(attitude_controller_t *ac, double t) {
  drone_state_set_target(ac->yaw_state, t);
}
void attitude_controller_set_alt_target(attitude_controller_t *ac, double t) {
  drone_state_set_target(ac->alt_state, t);
}

void attitude_controller_set_targets(attitude_controller_t *ac, double *targs) {
  drone_state_set_target(ac->roll_state, targs[0]);
  drone_state_set_target(ac->pitch_state, targs[1]);
  drone_state_set_target(ac->yaw_state, targs[2]);
  drone_state_set_target(ac->alt_state, targs[3]);
}

double attitude_controller_get_roll_target(attitude_controller_t *ac) {
  return drone_state_get_target(ac->roll_state);
}

double attitude_controller_get_roll_error(attitude_controller_t *ac) {
  return drone_state_get_error(ac->roll_state);
}

double attitude_controller_get_roll_state(attitude_controller_t *ac) {
  return drone_state_get_state(ac->roll_state);
}

double
attitude_controller_get_roll_state_derivative(attitude_controller_t *ac) {
  return drone_state_get_state_derivative(ac->roll_state);
}

double
attitude_controller_get_roll_error_derivative(attitude_controller_t *ac) {
  return drone_state_get_error_derivative(ac->roll_state);
}

double attitude_controller_get_roll_error_integral(attitude_controller_t *ac) {
  return drone_state_get_error_integral(ac->roll_state);
}

double attitude_controller_get_pitch_target(attitude_controller_t *ac) {
  return drone_state_get_target(ac->pitch_state);
}

double attitude_controller_get_pitch_error(attitude_controller_t *ac) {
  return drone_state_get_error(ac->pitch_state);
}

double attitude_controller_get_pitch_state(attitude_controller_t *ac) {
  return drone_state_get_state(ac->pitch_state);
}

double
attitude_controller_get_pitch_state_derivative(attitude_controller_t *ac) {
  return drone_state_get_state_derivative(ac->pitch_state);
}

double
attitude_controller_get_pitch_error_derivative(attitude_controller_t *ac) {
  return drone_state_get_error_derivative(ac->pitch_state);
}

double attitude_controller_get_pitch_error_integral(attitude_controller_t *ac) {
  return drone_state_get_error_integral(ac->pitch_state);
}

double attitude_controller_get_yaw_target(attitude_controller_t *ac) {
  return drone_state_get_target(ac->yaw_state);
}

double attitude_controller_get_yaw_error(attitude_controller_t *ac) {
  return drone_state_get_error(ac->yaw_state);
}

double attitude_controller_get_yaw_state(attitude_controller_t *ac) {
  return drone_state_get_state(ac->yaw_state);
}

double attitude_controller_get_yaw_state_derivative(attitude_controller_t *ac) {
  return drone_state_get_state_derivative(ac->yaw_state);
}

double attitude_controller_get_yaw_error_derivative(attitude_controller_t *ac) {
  return drone_state_get_error_derivative(ac->yaw_state);
}

double attitude_controller_get_yaw_error_integral(attitude_controller_t *ac) {
  return drone_state_get_error_integral(ac->yaw_state);
}

double attitude_controller_get_alt_target(attitude_controller_t *ac) {
  return drone_state_get_target(ac->alt_state);
}

double attitude_controller_get_alt_error(attitude_controller_t *ac) {
  return drone_state_get_error(ac->alt_state);
}

double attitude_controller_get_alt_state(attitude_controller_t *ac) {
  return drone_state_get_state(ac->alt_state);
}

double attitude_controller_get_alt_state_derivative(attitude_controller_t *ac) {
  return drone_state_get_state_derivative(ac->alt_state);
}

double attitude_controller_get_alt_error_derivative(attitude_controller_t *ac) {
  return drone_state_get_error_derivative(ac->alt_state);
}

double attitude_controller_get_alt_error_integral(attitude_controller_t *ac) {
  return drone_state_get_error_integral(ac->alt_state);
}
