#include <math.h>
#include <rc/math/ring_buffer.h>

#include "state.h"

void drone_state_init(drone_state_t *ds, int s, double r) {
  rc_ringbuf_alloc(ds->state, s);
  rc_ringbuf_alloc(ds->state_derivative, s);
  rc_ringbuf_alloc(ds->error, s);
  rc_ringbuf_alloc(ds->error_integral, s);
  rc_ringbuf_alloc(ds->error_derivative, s);

  ds->size = s;
  ds->rate = r;
  ds->target = 0;
}

void drone_state_destroy(drone_state_t *ds) {
  rc_ringbuf_free(ds->state);
  rc_ringbuf_free(ds->state_derivative);
  rc_ringbuf_free(ds->error);
  rc_ringbuf_free(ds->error_integral);
  rc_ringbuf_free(ds->error_derivative);
}

void drone_state_update(drone_state_t *ds, double s) {
  double e, ed, ei, sd;
  e = ds->target - s;
  rc_ringbuf_insert(ds->state, s);
  rc_ringbuf_insert(ds->error, e);

  sd =
      ds->rate / ds->size * (s - rc_ringbuf_get_value(ds->state, ds->size - 1));
  ed = -sd;
  rc_ringbuf_insert(ds->state_derivative, sd);
  rc_ringbuf_insert(ds->error_derivative, ed);

  ei = e / ds->rate + rc_ringbuf_get_value(ds->error_integral, 0);
  rc_ringbuf_insert(ds->error_integral, ei);
}

double drone_state_derive(rc_ringbuf_t *rb, double rate) {
  return rate / rb->size *
         (rc_ringbuf_get_value(rb, 0) - rc_ringbuf_get_value(rb, rb->size - 1));
}

void drone_state_set_target(drone_state_t *ds, double t) { ds->target = t; }

double drone_state_get_target(drone_state_t *ds) { return ds->target; }
double drone_state_get_rate(drone_state_t *ds) { return ds->rate; }

double drone_state_get_state(drone_state_t *ds) {
  return rc_ringbuf_get_value(ds->state, 0);
}
double drone_state_get_error(drone_state_t *ds) {
  return rc_ringbuf_get_value(ds->error, 0);
}
double drone_state_get_state_derivative(drone_state_t *ds) {
  double res = 0;
  for( int i = 0; i < ds->state_derivative->size; ++i)
    res += rc_ringbuf_get_value(ds->state_derivative, i);
  return res/ds->size;
}
double drone_state_get_error_derivative(drone_state_t *ds) {
  double res = 0;
  for( int i = 0; i < ds->error_derivative->size; ++i)
    res += rc_ringbuf_get_value(ds->error_derivative, i);
  return res/ds->size;
}
double drone_state_get_error_integral(drone_state_t *ds) {
  return rc_ringbuf_get_value(ds->error_integral, 0);
}
