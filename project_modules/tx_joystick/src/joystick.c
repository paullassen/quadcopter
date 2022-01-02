/**
 * Author: Jason White
 *
 * Description:
 * Reads joystick/gamepad events and displays them.
 *
 * Compile:
 * gcc joystick.c -o joystick
 *
 * Run:
 * ./joystick [/dev/input/jsX]
 *
 * See also:
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <stdlib.h> //exit(0);
#include <string.h> //memset
#include <sys/socket.h>
#include <unistd.h>

#define SERVER "192.168.86.53"
//#define SERVER "192.168.1.149"
#define BUFLEN 14 // Max length of buffer
#define PORT 8888 // The port on which to send data

void die(char *s) {
  perror(s);
  exit(1);
}

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event) {
  ssize_t bytes;

  bytes = read(fd, event, sizeof(*event));

  if (bytes == sizeof(*event))
    return 0;

  /* Error, could not read full event. */
  return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd) {
  __u8 axes;

  if (ioctl(fd, JSIOCGAXES, &axes) == -1)
    return 0;

  return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd) {
  __u8 buttons;
  if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
    return 0;

  return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
  short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3]) {
  size_t axis = event->number / 2;

  if (axis < 3) {
    if (event->number % 2 == 0)
      axes[axis].x = event->value;
    else
      axes[axis].y = event->value;
  }

  return axis;
}

int main(int argc, char *argv[]) {
  struct sockaddr_in si_other;
  int s, slen = sizeof(si_other);

  if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    die("socket");
  }

  memset((char *)&si_other, 0, sizeof(si_other));
  si_other.sin_family = AF_INET;
  si_other.sin_port = htons(PORT);

  if (inet_aton(SERVER, &si_other.sin_addr) == 0) {
    fprintf(stderr, "inet_aton() failed\n");
    exit(1);
  }
  const char *device;
  int js;
  struct js_event event;

  if (argc > 1)
    device = argv[1];
  else
    device = "/dev/input/js1";

  js = open(device, O_RDONLY);
  int num_buttons, num_axes, num_channels;
  if (js == -1) {
    perror("Could not open joystick");
  } else {
    printf("Opened Joystick with %d buttons and %d axes\n",
           num_buttons = get_button_count(js), num_axes = get_axis_count(js));
  }
  num_buttons = 11;
  num_axes = 3;
  num_channels = num_buttons + 2 * num_axes;
  struct axis_state axes[num_axes];
  size_t axis;
  
  //set default controller vals
  axes[0].x = 0;
  axes[0].y = 0;
  axes[1].x = -32768;
  axes[1].y = 0;
  axes[2].x = 0;
  axes[2].y = -32768;
  

  int message[num_channels];
  int buf[num_channels];
  for (int j = 0; j < num_channels; j++){
    message[j] = 0;
    buf[j] = 0;
  }
  buf[13] = -32768;
  buf[16] = -32768;
  /* This loop will exit if the controller is unplugged. */
  int c = 0;
  while (read_event(js, &event) == 0) {
    switch (event.type) {
    case JS_EVENT_BUTTON:
      message[event.number] = event.value;
      break;
    case JS_EVENT_AXIS:
      axis = get_axis_state(&event, axes);
      if (axis < num_axes) {
        message[2 * axis + num_buttons] = axes[axis].x;
        message[2 * axis + num_buttons + 1] = axes[axis].y;
      }
      break;
    default:
      /* Ignore init events. */
      break;
    }
    // Ignore empty data
    int sum = 0;
    for( int i = 0; i < num_channels; ++i ){
      if ( message[i] != 0 ) ++sum;
    }
    if  (sum != 0 ) {
      for( int i = 0; i < num_channels; ++i ){
        buf[i] = message[i];
      }
    } 

    if (sendto(s, buf, sizeof(buf), 0, (struct sockaddr *)&si_other,
               slen) == -1) {
      die("sendto()");
    }

    ++c;
    fflush(stdout);
  }

  close(js);
  return 0;
}
