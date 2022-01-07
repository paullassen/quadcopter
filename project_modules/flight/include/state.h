#include <robotcontrol.h>


typedef struct {
  rc_ringbuf_t state[1]; 
  rc_ringbuf_t state_derivative[1];
  rc_ringbuf_t error[1];

  double target;

  int size;
  double rate;
} drone_state_t;


void drone_state_init(drone_state_t * ds, int s, double r);
void drone_state_destroy(drone_state_t * ds);

void drone_state_update(drone_state_t * ds, double s);

void drone_state_set_target(drone_state_t * ds, double t);

