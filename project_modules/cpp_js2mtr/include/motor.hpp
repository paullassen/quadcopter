#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

#define PRU_ADDR 0x4A300000 // Start of PRU memory Page 184 am335x TRM
#define PRU_LEN 0x80000     // Length of PRU memory
#define PRU0_DRAM 0x00000   // Offset to DRAM
#define PRU1_DRAM 0x02000
#define PRU_SHAREDMEM 0x10000 // Offset to shared memory

#define MAXCH 4
#define PWM_MIN 5000
#define PWM_MAX 11000

typedef struct {
  uint32_t *pulse_width;
  unsigned int *pru;
  unsigned int *pru0DRAM_32int_ptr; // Points to the start of local DRAM
  unsigned int *pru1DRAM_32int_ptr; // Points to the start of local DRAM
  unsigned int
      *prusharedMem_32int_ptr; // Points to the start of the shared memory

} motor_interface_t;

int init_motor_interface(motor_interface_t *m);
int destroy_motor_interface(motor_interface_t *m);
void write_motor_pw(motor_interface_t *m,
                    unsigned int *pw); // Write Motor Value by Pulse Width
void write_motor_pct(motor_interface_t *m, double *pct);

#endif
