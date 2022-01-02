/* 
 *
 *  pwm tester
 *	The on cycle and off cycles are stored in each PRU's Data memory
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>

#define MAXCH 4

#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU0_DRAM		0x00000			// Offset to DRAM
#define PRU1_DRAM		0x02000
#define PRU_SHAREDMEM		0x10000			// Offset to shared memory

#define PWM_MIN	4617
#define PWM_MAX 12309

unsigned int	*pru0DRAM_32int_ptr;		// Points to the start of local DRAM
unsigned int	*pru1DRAM_32int_ptr;		// Points to the start of local DRAM
unsigned int	*prusharedMem_32int_ptr;	// Points to the start of the shared memory

int main(int argc, char *argv[])
{
	unsigned int	*pru;		// Points to start of PRU memory.
	int	fd;
	int 	pwm;
	int 	ch;
	printf("Pulse Test\n");
	
	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		printf ("ERROR: could not open /dev/mem.\n\n");
		return 1;
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED) {
		printf ("ERROR: could not map memory.\n\n");
		return 1;
	}
	close(fd);
	printf ("Using /dev/mem.\n");
	
	pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =     pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;	// Points to start of shared memory

	int i;
	for(i = 0; i < 10; ++i){
		struct timespec ts;
		ts.tv_sec  = 10/1000;
		ts.tv_nsec = 10000000;
		
		for(pwm = PWM_MIN; pwm <= PWM_MAX; pwm+=77){
			for(ch = 0; ch < 8; ch++)
				pru1DRAM_32int_ptr[ch] = pwm;
			nanosleep(&ts,&ts);
		}
		
		for(ch = 0; ch < 8; ch++)
			pru1DRAM_32int_ptr[ch] = PWM_MAX;

		for(pwm = PWM_MAX; pwm >= PWM_MIN; pwm-=77){
			for(ch = 0; ch < 8; ch++)
				pru1DRAM_32int_ptr[ch] = pwm;
			nanosleep(&ts,&ts);
		}

		for(ch = 0; ch < 8; ch++)
			pru1DRAM_32int_ptr[ch] = PWM_MIN;
	}
	if(munmap(pru, PRU_LEN)) {
		printf("munmap failed\n");
	} else {
		printf("munmap succeeded\n");
	}
}

