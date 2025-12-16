#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84E6);
//音符频率定义
const int P0 = 0;  // 休止符

const int L1 = 262;
const int L2 = 294;
const int L3 = 330;
const int L4 = 349;
const int L5 = 392;
const int L6 = 440;
const int L7 = 494;

const int M1 = 523;
const int M2 = 587;
const int M3 = 659;
const int M4 = 698;
const int M5 = 784;
const int M6 = 880;
const int M7 = 988;

const int H1 = 1047;
const int H2 = 1175;
const int H3 = 1319;
const int H4 = 1397;
const int H5 = 1568;
const int H6 = 1760;
const int H7 = 1976;
extern "C" void Buzzer_Task()
{
  buzzer.set(5000, 0.1);

  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  while (true) {
    osDelay(100);
  }
}
