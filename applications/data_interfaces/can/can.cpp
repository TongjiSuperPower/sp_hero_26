#include "can.hpp"

#include "controllers/chassis_controller/chassis_task.hpp"
#include "controllers/gimbal_controller/gimbal_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"

//chassis CAN2（4个）
void chassis_send()
{
  wheel_lf.write(can2.tx_data);
  wheel_lr.write(can2.tx_data);
  wheel_rf.write(can2.tx_data);
  wheel_rr.write(can2.tx_data);
  can2.send(wheel_lf.tx_id);
}
