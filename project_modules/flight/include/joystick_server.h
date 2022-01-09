#ifndef JOYSTICK_SERVER_H
#define JOYSTICK_SERVER_H

#include <pthread.h>

#define JS_RAW_LEN 17 // Max length of buffer
#define JS_PORT 8888  // The port on which to listen for incoming data
#define JS_LEN JS_RAW_LEN - 1
#define JS_EMPTY_CHANNEL 8

typedef enum {
  JS_A_BUTTON = 0,
  JS_B_BUTTON = 1,
  JS_X_BUTTON = 2,
  JS_Y_BUTTON = 3,
  JS_L_BUTTON = 4,
  JS_R_BUTTON = 5,
  JS_START_BUTTON = 6,
  JS_SELECT_BUTTON = 7,
  JS_L_STICK_BUTTON = 8,
  JS_R_STICK_BUTTON = 9,
  JS_L_STICK_LR = 10,
  JS_L_STICK_UD = 11,
  JS_L_BUMPER = 12,
  JS_R_STICK_LR = 13,
  JS_R_STICK_UD = 14,
  JS_R_BUMPER = 15
} joystick_channel_t;

typedef struct {
  int channel[JS_LEN];
  double norm_channel[JS_LEN];

  pthread_t pid;
  int running;
  int initialized;
} joystick_t;

void joystick_init(joystick_t *js);
void joystick_start(joystick_t *js);
void joystick_stop(joystick_t *js);
void joystick_update(joystick_t *js, int *buf);

int joystick_get_raw(joystick_t *js, joystick_channel_t ch);
double joystick_get_norm(joystick_t *js, joystick_channel_t ch);
void joystick_print(joystick_t *js);
void joystick_print_header();

void *joystick_thread(void *joystick);
#endif
