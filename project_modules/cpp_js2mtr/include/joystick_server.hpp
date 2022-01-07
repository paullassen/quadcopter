#ifndef JOYSTICK_SERVER_H
#define JOYSTICK_SERVER_H

#include <pthread.h>

#define JS_RAW_LEN 17 // Max length of buffer
#define PORT 8888 // The port on which to listen for incoming data
#define JS_LEN JS_RAW_LEN-1

typedef enum {
  JS_A_BUTTON       =  0, 
  JS_B_BUTTON       =  1,
  JS_X_BUTTON       =  2, 
  JS_Y_BUTTON       =  3, 
  JS_L_BUTTON       =  4, 
  JS_R_BUTTON       =  5, 
  JS_START_BUTTON   =  6, 
  JS_SELECT_BUTTON  =  7, 
  JS_L_STICK_BUTTON =  8, 
  JS_R_STICK_BUTTON =  9, 
  JS_L_STICK_LR     =  10,
  JS_L_STICK_UD     =  11,
  JS_L_BUMPER       =  12,
  JS_R_STICK_LR     =  13,
  JS_R_STICK_UD     =  14,
  JS_R_BUMPER       =  15
} js_channel_t;

typedef enum {
  JS_RAW_A_BUTTON       =  0,
  JS_RAW_B_BUTTON       =  1,
  JS_RAW_X_BUTTON       =  2,
  JS_RAW_Y_BUTTON       =  3,
  JS_RAW_L_BUTTON       =  4,
  JS_RAW_R_BUTTON       =  5,
  JS_RAW_START_BUTTON   =  6,
  JS_RAW_SELECT_BUTTON  =  7,
  JS_RAW_L_STICK_BUTTON =  9,
  JS_RAW_R_STICK_BUTTON =  10,
  JS_RAW_L_STICK_LR     =  11,
  JS_RAW_L_STICK_UD     =  12,
  JS_RAW_L_BUMPER       =  13,
  JS_RAW_R_STICK_LR     =  14,
  JS_RAW_R_STICK_UD     =  15,
  JS_RAW_R_BUMPER       =  16
} js_raw_channel_t;

class Joystick {
  private:
    pthread_rwlock_t rwlock;
    int channel[JS_LEN];
    double norm_channel[JS_LEN];

  public:
    Joystick();
    ~Joystick();
    void write(int * buf);
    int get_channel(js_channel_t);
    double get_norm_channel(js_channel_t);

    void print();
    void print_norm();
    void print_header();
};

void *joystick_server_thread(void *thread_args);

#endif
