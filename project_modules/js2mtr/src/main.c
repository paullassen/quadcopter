#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "joystick_server.h"
#include "motor.h"

int main() {
  pthread_t js_receiver;
  joystick_t js[1];
  init_joystick(js);
  printf("\nLine 13\n");
  motor_interface_t m;
  init_motor_interface(&m);
  int running = 1;
  printf("\nLine 17\n");
  void **thread_arg = malloc(sizeof(void *) * 2);
  thread_arg[0] = (void *)js;
  thread_arg[1] = (void *)&running;
  int buf[JS_LEN];
  double norm_buf[JS_LEN];
  double mtr[4] = {0};

  read_joystick(js, buf);
  norm_joystick(buf, norm_buf);
  pthread_create(&js_receiver, NULL, js_thread, thread_arg);
  printf("\nLine 30\n");
  usleep(10000);
  while (running) {
    read_joystick(js, buf);
    norm_joystick(buf, norm_buf);
    print_joystick_norm(norm_buf);

    if (buf[6] == 1) // End on start_button press
      running = 0;

    for (int j = 0; j < 4; ++j) {
      if (buf[j]) {
        mtr[j] = norm_buf[R_BUMPER];
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
  pthread_join(js_receiver, NULL);
  printf("\nFinished Reading\n");
  print_joystick_norm(norm_buf);
  printf("\n");
  return 0;
}
