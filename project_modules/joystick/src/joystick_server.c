#include "joystick_server.h"
#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>  //printf
#include <stdlib.h> //exit(0);
#include <string.h> //memset
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>


void init_joystick(joystick_t *js){
  js->rwlock = malloc(sizeof(pthread_rwlock_t));
  if (js->rwlock == NULL)
    return NULL;
  pthread_rwlock_init(js->rwlock, NULL);
  js->a_button = 0;
  js->b_button = 0;
  js->x_button = 0;
  js->y_button = 0;

  js->l_button = 0;
  js->r_button = 0;

  js->start_button = 0;
  js->select_button = 0;

  js->l_stick_button = 0;
  js->r_stick_button = 0;

  js->l_stick_lr = 0;
  js->l_stick_ud = 0;
  js->l_bumper = 0;

  js->r_stick_lr = 0;
  js->r_stick_ud = 0;
  js->r_bumper = 0;
}

void kill_joystick(joystick_t *js){
  pthread_rwlock_destroy(js->rwlock);
  free(js->rwlock);
}

void read_joystick(joystick_t *js, int *buf){
  pthread_rwlock_rdlock(js->rwlock);
  buf[0] = js->a_button;
  buf[1] = js->b_button;
  buf[2] = js->x_button;
  buf[3] = js->y_button;
  buf[4] = js->l_button;
  buf[5] = js->r_button;
  buf[6] = js->start_button;
  buf[7] = js->select_button;
  buf[8] = js->l_stick_button;
  buf[9] = js->r_stick_button;
  
  buf[10] = js->l_stick_lr;
  buf[11] = js->l_stick_ud;
  buf[12] = js->l_bumper;

  buf[13] = js->r_stick_lr;
  buf[14] = js->r_stick_ud;
  buf[15] = js->r_bumper;
  pthread_rwlock_unlock(js->rwlock);
}

void norm_joystick(int *buf, double *norm_buf){
  for( int i = 0; i < JS_LEN; ++i ){
    if( i < 10 ){
      norm_buf[i] = (double) buf[i];
    } else if( i==12 || i ==15 ){
      norm_buf[i] = (double) buf[i]/65536 +0.5;
    } else {
      norm_buf[i] = (double) buf[i]/ (65536/2);
    }
  }
}

void write_joystick(joystick_t *js, int *buf) {
  pthread_rwlock_wrlock(js->rwlock);
  js->a_button = buf[0];
  js->b_button = buf[1];
  js->x_button = buf[2];
  js->y_button = buf[3];

  js->l_button = buf[4];
  js->r_button = buf[5];

  js->start_button = buf[6];
  js->select_button = buf[7];

  js->l_stick_button = buf[9];
  js->r_stick_button = buf[10];

  js->l_stick_lr = buf[11];
  js->l_stick_ud = buf[12];
  js->l_bumper = buf[13];

  js->r_stick_lr = buf[14];
  js->r_stick_ud = buf[15];
  js->r_bumper = buf[16];
  pthread_rwlock_unlock(js->rwlock);
}

void print_joystick(joystick_t *js) {
  pthread_rwlock_rdlock(js->rwlock);
  printf("%d\t", js->a_button);
  printf("%d\t", js->b_button);
  printf("%d\t", js->x_button);
  printf("%d\t", js->y_button);
  printf("%d\t", js->l_button);
  printf("%d\t", js->r_button);
  printf("%d\t", js->start_button);
  printf("%d\t", js->select_button);
  printf("%d\t", js->l_stick_button);
  printf("%d\t", js->r_stick_button);
  printf("%d\t", js->l_stick_lr);
  printf("%d\t", js->l_stick_ud);
  printf("%d\t", js->r_stick_lr);
  printf("%d\t", js->r_stick_ud);
  printf("%d\t", js->l_bumper);
  printf("%d\t", js->r_bumper);
  printf("\r");
  pthread_rwlock_unlock(js->rwlock);
}

void print_header() {
  printf("\n");
  printf("%s\t", "a");
  printf("%s\t", "b");
  printf("%s\t", "x");
  printf("%s\t", "y");
  printf("%s\t", "l");
  printf("%s\t", "r");
  printf("%s\t", "start");
  printf("%s\t", "select");
  printf("%s\t", "l_stick");
  printf("%s\t", "r_stick");
  printf("%s\t", "l_lr");
  printf("%s\t", "l_ud");
  printf("%s\t", "r_lr");
  printf("%s\t", "r_ud");
  printf("%s\t", "l_bump");
  printf("%s\t", "r_bump");
  printf("\n");
}

void *js_thread(void *thread_args) {
  void **args = (void **)thread_args;
  joystick_t *js = (joystick_t *)*args;
  int *running = (int *)args[1];
  printf("Joystick Thread: Line 141\n");
  struct sockaddr_in si_me, si_other;

  int s;
  int slen = sizeof(si_other); 
  int recv_len;
  int buf[BUFLEN] = {0};
  write_joystick(js, buf);

  // create a UDP socket
  printf("Joystick Thread: Line 151\n");
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    pthread_exit(NULL);
  }

  // zero out the structure
  memset((char *)&si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  // bind socket to port
  printf("Joystick Thread: Line 164\n");
  if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
    pthread_exit(NULL);
  }
  // keep listening for data
  print_header();
  while (*running) {
    // printf("Waiting for data...");
    fflush(stdout);

    // try to receive some data, this is a blocking call
    if ((recv_len = recvfrom(s, buf, BUFLEN * 4, 0,
                             (struct sockaddr *)&si_other, &slen)) == -1) {
      pthread_exit(NULL);
    }
    print_joystick(js);
    write_joystick(js, buf);
  }

  close(s);
  pthread_exit(NULL);
}
