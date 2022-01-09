#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "joystick_server.h"
#include "motor.h"
#include "state.h"

int main() {
  pthread_t js_receiver;
  joystick_t js[1];
  joystick_init(js);
  printf("\nLine 13\n");
  motor_interface_t m;
  init_motor_interface(&m);
  int running = 1;
  printf("\nLine 17\n");
  void **thread_arg = malloc(sizeof(void *) * 2);
  thread_arg[0] = (void *)js;
  thread_arg[1] = (void *)&running;
  double mtr[4] = {0};

  joystick_start(js);
  joystick_print_header();
  printf("\nLine 30\n");
  usleep(10000);
  while (running) {
    joystick_print(js);

    if (joystick_get_raw(js,JS_START_BUTTON)) // End on start_button press
      running = 0;

    for (int j = 0; j < 4; ++j) {
      if (joystick_get_raw(js,j)) {
        mtr[j] = joystick_get_norm(js, JS_R_BUMPER);
      } else {
        mtr[j] = 0;
      }
    }
    write_motor_pct(&m, mtr);
    usleep(10000);
  }

  for (int i = 0; i < 4; ++i)
    mtr[i] = 0;

  write_motor_pct(&m, mtr);
  destroy_motor_interface(&m);
  running = 0;
  joystick_stop(js);
  printf("\nFinished Reading\n");
  joystick_print(js);
  printf("\n");
  return 0;
}
