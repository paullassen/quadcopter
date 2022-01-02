#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "joystick_server.h"

int main() {
  printf("Main Thread: Line 9\n");
  pthread_t js_receiver;
  joystick_t js;
  init_joystick(&js);
  int running = 1;

  void **thread_arg = malloc(sizeof(void *) * 2);
  thread_arg[0] = (void *)&js;
  thread_arg[1] = (void *)&running;
  printf("Main Thread: Line 17\n");
  pthread_create(&js_receiver, NULL, js_thread, thread_arg);

  printf("Main Thread: Line 20\n");
  sleep(20);
  running = 0;
  pthread_join(js_receiver, NULL);
  printf("\nFinished Reading\n");
  print_joystick(&js);
  printf("\n");
  return 0;
}
