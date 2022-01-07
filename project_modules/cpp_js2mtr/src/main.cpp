#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "joystick_server.hpp"
#include "motor.hpp"

int main() {
  pthread_t js_receiver;
  Joystick js;
  printf("\nLine 13\n");
  motor_interface_t m;
  init_motor_interface(&m);
  int running = 1;
  printf("\nLine 17\n");
  void **thread_arg = (void **)malloc(sizeof(void *) * 2);
  thread_arg[0] = (void *)&js;
  thread_arg[1] = (void *)&running;
  double mtr[4] = {0};

  pthread_create(&js_receiver, NULL, joystick_server_thread, thread_arg);
  printf("\nLine 30\n");
  usleep(10000);
  while (running) {
    js.print();
    if (js.get_channel(JS_START_BUTTON)) // End on start_button press
      running = 0;

    mtr[0] = (js.get_channel(JS_A_BUTTON) ? js.get_norm_channel(JS_R_BUMPER) : 0);
    mtr[1] = (js.get_channel(JS_B_BUTTON) ? js.get_norm_channel(JS_R_BUMPER) : 0);
    mtr[2] = (js.get_channel(JS_X_BUTTON) ? js.get_norm_channel(JS_R_BUMPER) : 0);
    mtr[3] = (js.get_channel(JS_Y_BUTTON) ? js.get_norm_channel(JS_R_BUMPER) : 0);

    write_motor_pct(&m, mtr);
    usleep(10000);
  }

  for (int i = 0; i < 4; ++i)
    mtr[i] = 0;

  write_motor_pct(&m, mtr);
  destroy_motor_interface(&m);
  running = 0;
  pthread_join(js_receiver, NULL);
  printf("\nFinished Reading\n");
  js.print();
  printf("\n");
  return 0;
}
