
#include "motor.h"
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

int init_motor_interface(motor_interface_t *m) {
  int fd;
  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd == -1) {
    printf("ERROR: Failed to open /dev/mem.\n\n");
    return -1;
  }
  m->pru = mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
  if (m->pru == MAP_FAILED) {
    printf("ERROR: Failed to map memory.\n\n");
    return -1;
  }
  close(fd);

  m->pru0DRAM_32int_ptr = m->pru + PRU0_DRAM / 4 + 0x200 / 4;
  m->pru1DRAM_32int_ptr = m->pru + PRU1_DRAM / 4 + 0x200 / 4;
  m->prusharedMem_32int_ptr = m->pru + PRU_SHAREDMEM / 4;

  m->pulse_width = m->pru1DRAM_32int_ptr;
  for (int i = 0; i < MAXCH; ++i) {
    m->pulse_width[i] = PWM_MIN;
  }
  return 0;
}

int destroy_motor_interface(motor_interface_t *m) {
  return munmap(m->pru, PRU_LEN);
}

void write_motor_pw(motor_interface_t *m, unsigned int *pw) {
  for (int i = 0; i < MAXCH; ++i) {
    m->pulse_width[i] = (uint32_t)pw[i];
  }
}

void write_motor_pct(motor_interface_t *m, double *pct) {
  unsigned int pw[4];
  for (int i = 0; i < MAXCH; ++i) {
    pw[i] = (uint32_t)(pct[i] * (PWM_MAX - PWM_MIN)) + PWM_MIN;
  }
  write_motor_pw(m, pw);
}
