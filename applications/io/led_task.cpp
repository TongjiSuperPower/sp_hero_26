#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);
int state = 0;
float r = 1, g = 0, blue = 0;

extern "C" void LED_Task()
{
  led.start();
  float step = 0.01f;

  while (true) {
    switch (state) {
      case 0:
        r -= step;
        g += step;
        if (r < 0.0f) {
          r = 0.0f;
          g = 1.0f;
          state = 1;
        }
        break;
      case 1:
        g -= step;
        blue += step;
        if (g < 0.0f) {
          g = 0.0f;
          blue = 1.0f;
          state = 2;
        }
        break;
      case 2:
        blue -= step;
        r += step;
        if (blue < 0.0f) {
          blue = 0.0f;
          r = 1.0f;
          state = 0;
        }
        break;
    }
    led.set(r, g, blue);
    osDelay(20);
  }
}