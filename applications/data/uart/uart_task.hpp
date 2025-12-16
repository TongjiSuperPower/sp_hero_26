#ifndef UART_TASK_HPP
#define UART_TASK_HPP
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
#include "referee/ui/ui.hpp"
// #include "referee/vt03/vt03.hpp"

inline sp::DBus remote(&huart3);
inline sp::PM02 pm02(&huart6);
extern bool game_start_flag;

#endif