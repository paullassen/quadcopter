#include "joystick_server.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>  //printf
#include <stdlib.h> //exit(0);
#include <string.h> //memset
#include <sys/socket.h>
#include <unistd.h>

void joystick_init(joystick_t *js) {

  for (int i = 0; i < JS_LEN; ++i) {
    if (i == JS_L_BUMPER || i == JS_R_BUMPER) {
      js->channel[i] = -32768;
    } else {
      js->channel[i] = 0;
    }
    js->norm_channel[i] = 0;
  }

  js->running = 0;
  js->initialized = 1;
}

void joystick_start(joystick_t *js) {
  if (!js->initialized) {
    joystick_init(js);
  }

  pthread_create(&js->pid, NULL, joystick_thread, (void *)js);
}

void joystick_stop(joystick_t *js) {
  js->running = 0;
  pthread_join(js->pid, NULL);
}

void joystick_update(joystick_t *js, int *buf) {
  for (int i = 0; i < JS_RAW_LEN; ++i) {
    if (i < JS_EMPTY_CHANNEL) {
      js->channel[i] = buf[i];
      js->norm_channel[i] = (double)buf[i];
    } else if (i > JS_EMPTY_CHANNEL) {
      js->channel[i - 1] = buf[i];
    }
  }

  for (int i = JS_EMPTY_CHANNEL; i < JS_LEN; ++i) {
    if (i == JS_L_BUMPER || i == JS_R_BUMPER) { // The Triggers/Bumpers
      js->norm_channel[i] = (double)js->channel[i] / 65536 + 0.5;
    } else if (i == JS_L_STICK_UD ||
               i == JS_R_STICK_UD) { // The Up/Down axis on the sticks
      js->norm_channel[i] = (double)-js->channel[i] / (65536 / 2);
    } else if (i == JS_L_STICK_LR ||
               i == JS_R_STICK_LR) { // The Left/Right axis on the sticks
      js->norm_channel[i] = (double)js->channel[i] / (65536 / 2);
    } else {
      js->norm_channel[i] = (double)js->channel[i];
    }
  }
}

int joystick_get_raw(joystick_t *js, joystick_channel_t ch) {
  return js->channel[ch];
}

double joystick_get_norm(joystick_t *js, joystick_channel_t ch) {
  if (js->norm_channel[ch] < 0.05 && js->norm_channel[ch] > -0.05) {
    return 0;
  }
  return js->norm_channel[ch];
}

void joystick_print(joystick_t *js) {
  for (int i = 0; i < JS_LEN; ++i) {
    printf("%0.2f  ", js->norm_channel[i]);
  }
  printf("\r");
}

void joystick_print_header() {
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

void *joystick_thread(void *joystick) {
  joystick_t *js = (joystick_t *)joystick;
  js->running = 1;
  printf("Joystick Thread: Line 141\n");
  struct sockaddr_in si_me, si_other;

  int s;
  unsigned int slen = sizeof(si_other);
  int recv_len;
  int buf[JS_RAW_LEN] = {0};
  buf[JS_R_BUMPER + 1] = -32768;
  buf[JS_L_BUMPER + 1] = -32768;

  // create a UDP socket
  printf("Joystick Thread: Line 151\n");
  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    pthread_exit(NULL);
  }

  // zero out the structure
  memset((char *)&si_me, 0, sizeof(si_me));

  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(JS_PORT);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);

  // bind socket to port
  printf("Joystick Thread: Line 164\n");
  if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
    pthread_exit(NULL);
  }
  // keep listening for data
  while (js->running) {
    // printf("Waiting for data...");
    fflush(stdout);

    // try to receive some data, this is a blocking call
    if ((recv_len = recvfrom(s, buf, JS_RAW_LEN * 4, 0,
                             (struct sockaddr *)&si_other, &slen)) == -1) {
      pthread_exit(NULL);
    }
    joystick_update(js, buf);
  }

  close(s);
  pthread_exit(NULL);
}
