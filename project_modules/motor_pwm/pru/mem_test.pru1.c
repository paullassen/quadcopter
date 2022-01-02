// This code does MAXCH parallel PWM channels.
// It's period is 3 us
#include "resource_table_empty.h"
#include <pru_cfg.h>
#include <stdint.h>

#define PRU1_DRAM 0x00000
volatile unsigned int *pru1_dram = (unsigned int *)(PRU1_DRAM + 0x200);

#define MAXCH 8
#define PERIOD 102576 // 50 hz
#define PWM_MAX 11000
#define PWM_MIN  5000

volatile register uint32_t __R30;
volatile register uint32_t __R31;

void main(void) {
  uint32_t ch;
  uint32_t pin[] = {8, 10, 9, 11, 6, 7, 4, 5};
  uint32_t period = PERIOD;
  uint32_t count = 0;
  /* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
  CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

  for (ch = 0; ch < MAXCH; ch++) {
    pru1_dram[ch] = PWM_MAX;
  }

  pru1_dram[MAXCH] = PERIOD;

  while (1) {
    uint32_t tmp = __R30;
    if (count++ >= pru1_dram[MAXCH]) {
      tmp |= 0xFF << 4;
      count = 0;
    } else {
      if (count == pru1_dram[0])
        tmp &= ~(0x1 << pin[0]);
      if (count == pru1_dram[1])
        tmp &= ~(0x1 << pin[1]);
      if (count == pru1_dram[2])
        tmp &= ~(0x1 << pin[2]);
      if (count == pru1_dram[3])
        tmp &= ~(0x1 << pin[3]);
      if (count == pru1_dram[4])
        tmp &= ~(0x1 << pin[4]);
      if (count == pru1_dram[5])
        tmp &= ~(0x1 << pin[5]);
      if (count == pru1_dram[6])
        tmp &= ~(0x1 << pin[6]);
      if (count == pru1_dram[7])
        tmp &= ~(0x1 << pin[7]);
    }
    __R30 = tmp;
  }
}
