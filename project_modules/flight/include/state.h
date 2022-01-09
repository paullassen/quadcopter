#ifndef _QUADCOPTER_STATE_H_
#define _QUADCOPTER_STATE_H_

#include <rc/math/ring_buffer.h>

typedef struct {
  rc_ringbuf_t state[1];
  rc_ringbuf_t state_derivative[1];

  rc_ringbuf_t error[1];
  rc_ringbuf_t error_integral[1];
  rc_ringbuf_t error_derivative[1];

  double target;

  int size;
  double rate;
} drone_state_t;

void drone_state_init(drone_state_t *ds, int s, double r);
void drone_state_destroy(drone_state_t *ds);

void drone_state_update(drone_state_t *ds, double s);

void drone_state_set_target(drone_state_t *ds, double t);

double drone_state_get_target(drone_state_t *ds);
double drone_state_get_rate(drone_state_t *ds);

double drone_state_get_state(drone_state_t *ds);
double drone_state_get_error(drone_state_t *ds);
double drone_state_get_state_derivative(drone_state_t *ds);
double drone_state_get_error_derivative(drone_state_t *ds);
double drone_state_get_error_integral(drone_state_t *ds);

#endif
