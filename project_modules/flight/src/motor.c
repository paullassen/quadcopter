
#include "motor.h"
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <rc/adc.h>
#include <rc/math/vector.h>

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
  rc_adc_init();

  double k = 0.0991;
  double b = k/20;
  double lr = 0.27;
  double mass = 1.05;
  double sqrt2 = 1.41421356237;

  rc_matrix_alloc(&m->MotorMixer, 4, 4);
  for(int i = 0; i < 4; ++i){
    m->MotorMixer.d[i][0] = 1/(4*k);
    m->MotorMixer.d[i][1] = sqrt2/(4*k*lr);
    m->MotorMixer.d[i][2] = sqrt2/(4*k*lr);
    m->MotorMixer.d[i][3] = 1/(4*b);
    if(i == 0 || i == 3)
      m->MotorMixer.d[i][1] *= -1;
    if(i == 0 || i == 1)
      m->MotorMixer.d[i][2] *= -1;
  }
  m->voltage = rc_adc_dc_jack();

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

void write_motor_volt(motor_interface_t * m, double *volt) {
  double pct[4];
  m->voltage = rc_adc_dc_jack();
  for (int i = 0; i < MAXCH; ++i) {
    pct[i] = volt[i]/m->voltage;
  }
  write_motor_pct(m, pct);
}

void write_motor_wrench(motor_interface_t * m, rc_vector_t  * w) {
  rc_vector_t motor_volt;
  rc_vector_alloc(&motor_volt, 4);
  rc_matrix_times_col_vec(m->MotorMixer, *w, &motor_volt);
  
  write_motor_volt(m,motor_volt.d);
  rc_vector_free(&motor_volt);
}
