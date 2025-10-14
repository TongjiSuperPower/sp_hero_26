#ifndef _BUZZER_TASK_HPP_
#define _BUZZER_TASK_HPP_

#include <cstdint>

#include "calibrate_task.hpp"
#include "controllers/detect_task.hpp"
#include "main.h"

//dragon 天空之城 节拍曲调宏定义
#define ZERO 3000  //
#define R 1000000  //F_CLOCK/(psc+1)=50M/(49+1)=1000000
//低音区
constexpr uint16_t L1 = 262;
constexpr uint16_t half_L1 = 277;
constexpr uint16_t L2 = 294;
constexpr uint16_t half_L2 = 311;
constexpr uint16_t L3 = 330;
constexpr uint16_t L4 = 349;
constexpr uint16_t half_L4 = 370;
constexpr uint16_t L5 = 392;
constexpr uint16_t half_L5 = 410;
constexpr uint16_t L6 = 440;
constexpr uint16_t half_L6 = 466;
constexpr uint16_t L7 = 494;
//中音区
constexpr uint16_t M1 = 523;
constexpr uint16_t half_M1 = 554;
constexpr uint16_t M2 = 587;
constexpr uint16_t half_M2 = 622;
constexpr uint16_t M3 = 659;
constexpr uint16_t M4 = 698;
constexpr uint16_t half_M4 = 740;
constexpr uint16_t M5 = 784;
constexpr uint16_t half_M5 = 831;
constexpr uint16_t M6 = 880;
constexpr uint16_t half_M6 = 932;
constexpr uint16_t M7 = 988;
//高音区
constexpr uint16_t H1 = 1046;
constexpr uint16_t half_H1 = 1109;
constexpr uint16_t H2 = 1175;
constexpr uint16_t half_H2 = 1245;
constexpr uint16_t H3 = 1318;
constexpr uint16_t H4 = 1397;
constexpr uint16_t half_H4 = 1480;
constexpr uint16_t H5 = 1568;
constexpr uint16_t half_H5 = 1661;
constexpr uint16_t H6 = 1760;
constexpr uint16_t half_H6 = 1865;
constexpr uint16_t H7 = 1967;
//倍高音区
constexpr uint16_t HH1 = 2093;
constexpr uint16_t HH1_ = 2217;
constexpr uint16_t HH2 = 2349;
constexpr uint16_t HH2_ = 2489;
constexpr uint16_t HH3 = 2637;
constexpr uint16_t HH4 = 2794;
constexpr uint16_t HH4_ = 2960;
constexpr uint16_t HH5 = 3136;
constexpr uint16_t HH5_ = 3322;
constexpr uint16_t HH6 = 3520;
constexpr uint16_t HH6_ = 3729;
constexpr uint16_t HH7 = 3951;
//超高音区
constexpr uint16_t HHH1 = 4186;
constexpr uint16_t HHH1_ = 4435;
constexpr uint16_t HHH2 = 4699;
constexpr uint16_t HHH2_ = 4978;
constexpr uint16_t HHH3 = 5274;
constexpr uint16_t HHH4 = 5588;
constexpr uint16_t HHH4_ = 5920;
constexpr uint16_t HHH5 = 6272;
constexpr uint16_t HHH5_ = 6645;
constexpr uint16_t HHH6 = 7040;
constexpr uint16_t HHH6_ = 7458;
constexpr uint16_t HHH7 = 7902;

//极乐净土节拍曲调 宏定义
/* 以下部分是定义是把每个音符和频率值对应起来
 *  A+数字：表示音符；
 *  AH+数字：表示上面有点的那种音符；
 *  AL+数字：表示下面有点的那种音符
 */
#define A0 0
#define A1 441
#define A2 495
#define A3 556
#define A4 589
#define A5 661
#define A6 742

#define AL1 221
#define AL2 248
#define AL3 278
#define AL4 294
#define AL5 330
#define AL6 371
#define AL7 416

#define AH1 882
#define AH2 990
#define AH3 1112
#define AH4 1178
#define AH5 1322
#define AH6 1484
#define AH7 1665

/********** 这部分是用英文对应了拍子 **********/
#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625

//千本樱
//千本樱
//播放速度，值为四分音符的时长(ms)
#define SPEED 359
#define P 0  // 休止符

#endif