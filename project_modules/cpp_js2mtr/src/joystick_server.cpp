#include "joystick_server.hpp"
#include <arpa/inet.h>
#include <pthread.h>
#include <stdio.h>  //printf
#include <stdlib.h> //exit(0);
#include <string.h> //memset
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>


Joystick::Joystick(){
  pthread_rwlock_init(&rwlock, NULL);

  for( int i = 0; i < JS_LEN; ++i ){
    if( i == JS_L_BUMPER || i == JS_R_BUMPER ){
      channel[i] = -32768;
    } else {
      channel[i] = 0;
    }
  }
}

Joystick::~Joystick(){
  pthread_rwlock_destroy(&rwlock);
}

void Joystick::write(int * buf){
  pthread_rwlock_wrlock(&rwlock);
  // Buttons (exc. stick)
  for(int i = 0; i < 8; ++i){
    channel[i]  = buf[i];
    norm_channel[i] = (double) buf[i];
  }
  
  // Analog Inputs (inc. Stick Buttons)
  for(int i = 8; i < JS_LEN; ++i){
    channel[i] = buf[i+1];

    if( i==JS_L_BUMPER || i == JS_R_BUMPER ){ // The Triggers/Bumpers
      norm_channel[i] = (double) buf[i+1]/65536 +0.5;
    } else if( i==JS_L_STICK_UD || i ==JS_R_STICK_UD ){ // The Up/Down axis on the sticks
      norm_channel[i] = (double) -buf[i+1]/ (65536/2);
    } else if( i==JS_L_STICK_LR || i ==JS_R_STICK_LR ){ // The Left/Right axis on the sticks
      norm_channel[i] = (double) buf[i+1]/ (65536/2);
    } else {
      norm_channel[i] = (double) buf[i+1];
    }
  }
  pthread_rwlock_unlock(&rwlock);
}

int Joystick::get_channel(js_channel_t ch){
  return channel[ch];
}

double Joystick::get_norm_channel(js_channel_t ch){
  return norm_channel[ch];
}

void Joystick::print(){
  pthread_rwlock_rdlock(&rwlock);
  for(int i = 0; i < JS_LEN; ++i){
    printf("%0.2f  ", norm_channel[i]);
  }
  printf("\r");
  pthread_rwlock_unlock(&rwlock);  
}

void Joystick::print_header() {
  printf("\n");
  printf("%s  ", "a   ");
  printf("%s  ", "b   ");
  printf("%s  ", "x   ");
  printf("%s  ", "y   ");
  printf("%s  ", "l   ");
  printf("%s  ", "r   ");
  printf("%s  ", "st  ");
  printf("%s  ", "sel ");
  printf("%s  ", "l_s ");
  printf("%s  ", "r_s ");
  printf("%s  ", "l_lr");
  printf("%s  ", "l_ud");
  printf("%s  ", "l_bp");
  printf("%s  ", "r_lr");
  printf("%s  ", "r_ud");
  printf("%s  ", "r_bp");
  printf("\n");
}

void *joystick_server_thread(void *thread_args) {
  void **args = (void **)thread_args;
  Joystick *js = (Joystick  *)args[0];
  int *running = (int *)args[1];
  printf("Joystick Thread: Line 141\n");
  struct sockaddr_in si_me, si_other;

  int s;
  int slen = sizeof(si_other); 
  int recv_len;
  int buf[JS_RAW_LEN] = {0};
  buf[JS_RAW_L_BUMPER] = -32768;
  buf[JS_RAW_R_BUMPER] = -32768;

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
  js->print_header();
  while (*running) {
    // printf("Waiting for data...");
    fflush(stdout);

    // try to receive some data, this is a blocking call
    if ((recv_len = recvfrom(s, buf, JS_RAW_LEN * 4, 0,
                             (struct sockaddr *)&si_other, (socklen_t *)&slen)) == -1) {
      pthread_exit(NULL);
    }
    js->write(buf);
  }

  close(s);
  pthread_exit(NULL);
}
