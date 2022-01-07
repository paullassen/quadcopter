#include <math.h>
#include <rc/math/ring_buffer.h>

#include "state.h"

void drone_state_init(drone_state_t * ds, int s, double r){
  rc_ringbuf_alloc(ds->state, s);
  rc_ringbuf_alloc(ds->state_derivative, s);
  rc_ringbuf_alloc(ds->error, s);

  ds->size = s;
  ds->rate = r;
  ds->target = 0;
}

void drone_state_destroy(drone_state_t * ds){
  rc_ringbuf_free(ds->state);
  rc_ringbuf_free(ds->state_derivative);
  rc_ringbuf_free(ds->error);
}

void drone_state_update(drone_state_t * ds, double s){
  double sd;
  rc_ringbuf_insert(ds->state, s);
  rc_ringbuf_insert(ds->error, ds->target-s);
  sd = ds->rate/ds->size * (s - rc_ringbuf_get_value(ds->state, ds->size-1));
  rc_ringbuf_insert(ds->state_derivative, sd);
}

double drone_state_derive(rc_ringbuf_t * rb, double rate){
  return rate/rb->size * 
    (rc_ringbuf_get_value(rb, 0) - rc_ringbuf_get_value(rb, rb->size-1));
}

void drone_state_set_target(drone_state_t * ds, double t){
  ds->target = t;
}


