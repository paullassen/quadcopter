#ifndef JOYSTICK_SERVER_H
#define JOYSTICK_SERVER_H

#include <pthread.h>

#define BUFLEN 17 // Max length of buffer
#define PORT 8888 // The port on which to listen for incoming data
#define JS_LEN BUFLEN-1

#define A_BUTTON        0
#define B_BUTTON        1
#define X_BUTTON        2
#define Y_BUTTON        3
#define L_BUTTON        4
#define R_BUTTON        5
#define START_BUTTON    6
#define SELECT_BUTTON   7
#define L_STICK_BUTTON  8
#define R_STICK_BUTTON  9
#define L_STICK_LR      10
#define L_STICK_UD      11
#define L_BUMPER        12
#define R_STICK_LR      13
#define R_STICK_UD      14
#define R_BUMPER        15

typedef struct {
  int a_button;
  int b_button;
  int x_button;
  int y_button;

  int l_button;
  int r_button;

  int start_button;
  int select_button;

  int l_stick_button;
  int r_stick_button;

  int l_stick_ud;
  int l_stick_lr;
  int r_stick_ud;
  int r_stick_lr;

  int l_bumper;
  int r_bumper;
  
  pthread_rwlock_t * rwlock;
} joystick_t;

void init_joystick(joystick_t *js);
void read_joystick(joystick_t *js, int *buf);
void write_joystick(joystick_t *js, int *buf);
void print_joystick(joystick_t *js);
void print_header();
void *js_thread(void *thread_args);

#endif
