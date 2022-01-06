
#define MEAS_DEPTH 20

typedef struct {
  double data[MEAS_DEPTH];
  int index;
} circ_buf_t;

void init_circ_buf(circ_buf_t * buf) {
  buf->index = 0;
  for(int i = 0; i < MEAS_DEPTH; ++i) 
    data[i] = 0;
}


typedef struct {
  
} drone_state_t;
