#include "uart_task.hpp"
#include "io/imu_task.hpp"
#include "controllers/chassis_controller/chassis_task.hpp"
#include "controllers/shoot_controller/shoot_task.hpp"

// C板
sp::UI_Manager ui_manager;
using namespace sp::ui;
bool game_start_flag = false;

//电容
//字符串
String cap_str(Layer::LAYER_0, Color::CYAN, 2, 755, 150, 20, "CAP");
//电容总容量框:静态group1
Rectangle cap_rect(Layer::LAYER_0, Color::WHITE, 1, 600, 75, 1000, 100);
//电容容量:动态group2
Line cap_voltage(Layer::LAYER_0, Color::GREEN, 25, 600, 87, 1000, 87);
//电容低电量报警
Line cap_warning(Layer::LAYER_0, Color::PURPLE, 25, 600, 87, 1000, 87);

//总能量
//字符串
String energy_str(Layer::LAYER_1, Color::CYAN, 2, 1185, 150, 20, "ENERGY");
//能量容量框：静态group1
Rectangle energy_rect(Layer::LAYER_1, Color::WHITE, 1, 1100, 75, 1400, 100);
//能量容量：动态group2
Line energy_capacity(Layer::LAYER_1, Color::GREEN, 25, 1100, 87, 1400, 87);
//能量预警标识符：动态group3
Line energy_warning_line(Layer::LAYER_1, Color::PURPLE, 10, 617, 50, 617, 120);

//自瞄
//字符串：动态group5   掉线紫色，未识别橙色，正常青色
String autoaim_str(Layer::LAYER_3, Color::CYAN, 2, 30, 770, 20, "AUTOAIM");
//字符串：动态group6   掉线LOST
String autoaim_lost_str(Layer::LAYER_4, Color::PURPLE, 2, 220, 770, 20, "LOST");
//自瞄框：动态group4  自瞄档则开
Rectangle autoaim_rect(Layer::LAYER_2, Color::PURPLE, 5, 20, 740, 180, 780);

//摩擦轮
//字符串
String fric_str(Layer::LAYER_4, Color::CYAN, 2, 390, 880, 20, "FRIC");
//摩擦轮框：动态group4
Rectangle fric_rect(Layer::LAYER_4, Color::PURPLE, 5, 375, 850, 477, 890);

//射击状态
//连发字符串
String shoot_continue_str(Layer::LAYER_5, Color::CYAN, 2, 745, 880, 20, "CONTINUE");
//连发框：动态group4
Rectangle shoot_continue_rect(Layer::LAYER_8, Color::PURPLE, 5, 730, 850, 916, 890);
//单发字符串
String shoot_single_str(Layer::LAYER_5, Color::CYAN, 2, 565, 880, 20, "SINGLE");
//单发框：动态group4
Rectangle shoot_single_rect(Layer::LAYER_8, Color::PURPLE, 5, 545, 850, 699, 890);

//底盘状态
//底盘跟随字符串
String follow_str(Layer::LAYER_6, Color::CYAN, 2, 1050, 880, 20, "FOLLOW");
//底盘跟随框：动态group4
Rectangle follow_rect(Layer::LAYER_9, Color::PURPLE, 5, 1040, 850, 1180, 890);
//小陀螺字符串
String spin_str(Layer::LAYER_6, Color::CYAN, 2, 1230, 880, 20, "SPIN");
//小陀螺框：动态group4
Rectangle spin_rect(Layer::LAYER_9, Color::PURPLE, 5, 1220, 850, 1320, 890);

//电机掉线提示
//底盘
String chassis_dead_str(Layer::LAYER_3, Color::PURPLE, 2, 30, 690, 13, "CHASSIS");
//云台
String gimbal_dead_str(Layer::LAYER_3, Color::PURPLE, 2, 150, 690, 13, "GIMBAL");
//发射机构
String shoot_dead_str(Layer::LAYER_3, Color::PURPLE, 2, 250, 690, 13, "SHOOT");

//英雄复活UI
String hero_revive_str(Layer::LAYER_9, Color::PURPLE, 8, 1300, 800, 50, "HERO REVIVE");

//示宽线
//示宽线左：静态group1
Line width_line_left(Layer::LAYER_7, Color::PURPLE, 8, 490, 0, 678, 295);
//示宽线右：静态group1
Line width_line_right(Layer::LAYER_7, Color::PURPLE, 8, 1490, 0, 1327, 255);

//自瞄视野框
Rectangle autoaim_view_rect(Layer::LAYER_9, Color::WHITE, 2, 620, 300, 1200, 725);

//允许发弹量
//允许发弹量字
String projectile_allowance_str(Layer::LAYER_8, Color::CYAN, 2, 1450, 880, 20, "BULLET");
//允许发弹量数字: 动态group7
Integer projectile_allowance_num(Layer::LAYER_8, Color::PURPLE, 5, 1660, 880, 30, 0);

//RMUL补给区
String supply_str(Layer::LAYER_8, Color::CYAN, 2, 1260, 250, 20, "SUPPLY");

//哨兵血量字符
String sentry_blood_str(Layer::LAYER_9, Color::CYAN, 2, 30, 610, 20, "SENTRY BLOOD");
//哨兵血量数据
Integer sentry_blood_num(Layer::LAYER_9, Color::PURPLE, 2, 300, 610, 20, 0);
//哨兵提示符
// String sentry_blood_warning(Layer::LAYER_9, Color::PURPLE, 8, 1600, 800, 50, "SEN");

// //被墙卡住检测
// String stuck_str(Layer::LAYER_9, Color::PURPLE, 8, 1300, 800, 50, "STUCK");

//射速
//摩擦轮转速字符串
String fric_speed_str(Layer::LAYER_7, Color::CYAN, 2, 1480, 770, 20, "FRIC SPEED v-b+");
//摩擦轮转速数据：动态
Integer fric_speed_num(Layer::LAYER_7, Color::PURPLE, 2, 1775, 770, 20, 0);

uint64_t static_ui_num = 0;
uint64_t dynamic_ui_num = 0;

extern uint32_t autoaim_last_read_ms_;

extern "C" void UART_Task()
{
  pm02.request();
  remote.request();
  ui_manager.set_sender_id(pm02.robot_status.robot_id);  // 官方裁判系统(建议放在循环里)

  ui_manager.delete_all();
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  // 线和框们：group2
  ui_manager.pack(&cap_rect, &cap_voltage, &energy_capacity, &autoaim_rect, &energy_warning_line);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  //将摩擦轮置于down，射击模式置于连发
  fric_rect.set_operate_type(OperateType::DELETE);
  shoot_single_rect.set_operate_type(OperateType::DELETE);
  ui_manager.pack(
    &fric_rect, &shoot_single_rect, &shoot_single_rect, &follow_rect, &width_line_left,
    &width_line_right);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  //将底盘置于follow
  spin_rect.set_operate_type(OperateType::DELETE);
  ui_manager.pack(&spin_rect, &projectile_allowance_num);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  //允许发弹量
  ui_manager.pack(&projectile_allowance_str);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  //自瞄视野框
  ui_manager.pack(&autoaim_view_rect);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  //英雄复活
  ui_manager.pack(&hero_revive_str);
  pm02.send(ui_manager.data(), ui_manager.size());
  osDelay(33);

  while (true) {
    game_start_flag = (pm02.game_status.game_progress == 4);
    ui_manager.set_sender_id(pm02.robot_status.robot_id);
    osDelay(10);
    vis.autoaim_alive = (osKernelSysTick() - vis.autoaim_last_read_ms_ < 100);
    //静态UI
    if (static_ui_num < 2) {
      static_ui_num++;
    }
    if (static_ui_num == 2 && remote.keys.r) {
      static_ui_num = 0;
      //字符串们
      ui_manager.pack(&cap_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&energy_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&chassis_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&gimbal_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&shoot_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&follow_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&spin_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);

      ui_manager.pack(&fric_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&shoot_continue_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&shoot_single_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&projectile_allowance_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      // ui_manager.pack(&stuck_str);
      // pm02.send(ui_manager.data(), ui_manager.size());
      // osDelay(33);
      hero_revive_str.set_operate_type(OperateType::ADD);
      ui_manager.pack(&hero_revive_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&width_line_left, &width_line_right, &energy_rect, &cap_rect);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      ui_manager.pack(&energy_warning_line);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      energy_capacity.set_operate_type(OperateType::ADD);
      cap_voltage.set_operate_type(OperateType::ADD);
      ui_manager.pack(&energy_capacity, &cap_voltage);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      autoaim_view_rect.set_operate_type(OperateType::ADD);
      ui_manager.pack(&autoaim_view_rect);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      autoaim_str.set_operate_type(OperateType::ADD);
      ui_manager.pack(&autoaim_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      autoaim_lost_str.set_operate_type(OperateType::ADD);
      ui_manager.pack(&autoaim_lost_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);

      projectile_allowance_num.set_operate_type(OperateType::ADD);
      ui_manager.pack(&projectile_allowance_num);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      sentry_blood_str.set_operate_type(OperateType::ADD);
      ui_manager.pack(&sentry_blood_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      sentry_blood_num.set_operate_type(OperateType::ADD);
      ui_manager.pack(&sentry_blood_num);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      // sentry_blood_warning.set_operate_type(OperateType::ADD);
      // ui_manager.pack(&sentry_blood_warning);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);

      fric_speed_str.set_operate_type(OperateType::ADD);
      ui_manager.pack(&fric_speed_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
      fric_speed_num.set_operate_type(OperateType::ADD);
      ui_manager.pack(&fric_speed_num);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    //ui
    //电容电量线
    if (low_vol_flag) {
      cap_voltage.set_color(Color::GREEN);
    }
    else {
      cap_voltage.set_color(Color::ORANGE);
    }

    //满电26.7V
    cap_voltage.set_x2(
      600 + (super_cap.voltage * super_cap.voltage / (26.7 * 26.7)) * (1000 - 600));
    cap_voltage.set_operate_type(OperateType::MODIFY);

    //总能量
    if (energy_sum < 20000) {
      energy_capacity.set_x2(1100 + (20000 - energy_sum) / 20000 * 300);
    }
    else {
      energy_capacity.set_x2(1100);
    }
    energy_capacity.set_operate_type(OperateType::MODIFY);

    //RMUL补给区
    if (pm02.rfid_status.friendly_supply_no_trade) {
      supply_str.set_operate_type(OperateType::ADD);
    }
    else {
      supply_str.set_operate_type(OperateType::DELETE);
    }

    //自瞄
    //是否扫到/离线
    if (vis.autoaim_alive) {
      autoaim_str.set_color(Color::CYAN);
    }
    else {
      autoaim_str.set_color(Color::PURPLE);
    }
    autoaim_str.set_operate_type(OperateType::MODIFY);

    //自瞄掉线字符串
    if (vis.autoaim_alive) {
      autoaim_lost_str.set_operate_type(OperateType::DELETE);
    }
    else {
      autoaim_lost_str.set_operate_type(OperateType::ADD);
    }

    //框
    if (Gimbal_Mode == GIMBAL_AUTO) {
      autoaim_rect.set_operate_type(OperateType::ADD);
    }
    else {
      autoaim_rect.set_operate_type(OperateType::DELETE);
    }

    if (vis.control) {
      autoaim_view_rect.set_color(Color::PURPLE);
    }
    else {
      autoaim_view_rect.set_color(Color::WHITE);
    }
    autoaim_view_rect.set_operate_type(OperateType::MODIFY);
    //摩擦轮
    if (Fric_Mode == FRIC_DOWN || Fric_Mode == FRIC_OFF) {
      fric_rect.set_operate_type(OperateType::DELETE);
    }
    if (Fric_Mode == FRIC_ON) {
      fric_rect.set_operate_type(OperateType::ADD);
    }

    //射击状态
    if (Trigger_Mode == SHOOT_CLEAR || Trigger_Mode == SHOOT_INIT) {
      shoot_single_rect.set_operate_type(OperateType::DELETE);
      shoot_continue_rect.set_operate_type(OperateType::DELETE);
    }
    if (Trigger_Mode == SHOOT_READY_SINGLE) {
      shoot_single_rect.set_operate_type(OperateType::ADD);
      shoot_continue_rect.set_operate_type(OperateType::DELETE);
    }

    //底盘状态
    if (Chassis_Mode == CHASSIS_DOWN) {
      spin_rect.set_operate_type(OperateType::DELETE);
      follow_rect.set_operate_type(OperateType::DELETE);
    }
    if (Chassis_Mode == CHASSIS_FOLLOW) {
      spin_rect.set_operate_type(OperateType::DELETE);
      follow_rect.set_operate_type(OperateType::ADD);
    }
    if (Chassis_Mode == CHASSIS_SPIN) {
      spin_rect.set_operate_type(OperateType::ADD);
      follow_rect.set_operate_type(OperateType::DELETE);
    }

    //允许发弹量
    if (pm02.projectile_allowance.projectile_allowance_17mm < 65500) {
      projectile_allowance_num.set_value(pm02.projectile_allowance.projectile_allowance_17mm);
    }
    else {
      projectile_allowance_num.set_value(0);
    }
    projectile_allowance_num.set_operate_type(OperateType::MODIFY);

    //哨兵血量
    sentry_blood_num.set_operate_type(OperateType::MODIFY);

    if (
      pm02.robot_status.robot_id ==
      (sp::referee::robot_id::BLUE_STANDARD_3 || sp::referee::robot_id::BLUE_STANDARD_4)) {
      sentry_blood_num.set_value(pm02.game_robot_hp.blue_7_robot_hp);
    }
    if (
      pm02.robot_status.robot_id ==
      (sp::referee::robot_id::RED_STANDARD_3 || sp::referee::robot_id::RED_STANDARD_4)) {
      sentry_blood_num.set_value(pm02.game_robot_hp.red_7_robot_hp);
      //哨兵低血量提示符
      // if (pm02.game_robot_hp.red_7_robot_hp < 150) {
      //   sentry_blood_warning.set_operate_type(OperateType::ADD);
      // }
      // else {
      //   sentry_blood_warning.set_operate_type(OperateType::DELETE);
      // }
    }

    //英雄复活ui
    if (
      pm02.robot_status.robot_id ==
      (sp::referee::robot_id::BLUE_STANDARD_3 || sp::referee::robot_id::BLUE_STANDARD_4)) {
      static uint16_t last_hero_hp = pm02.game_robot_hp.red_1_robot_hp;
      if (pm02.game_robot_hp.red_1_robot_hp != 0 && last_hero_hp == 0) {
        hero_revive_str.set_operate_type(OperateType::ADD);
      }
      else {
        hero_revive_str.set_operate_type(OperateType::DELETE);
      }
      last_hero_hp = pm02.game_robot_hp.red_1_robot_hp;
    }

    if (
      pm02.robot_status.robot_id ==
      (sp::referee::robot_id::RED_STANDARD_3 || sp::referee::robot_id::RED_STANDARD_4)) {
      static uint16_t last_hero_hp = pm02.game_robot_hp.blue_1_robot_hp;
      if (pm02.game_robot_hp.blue_1_robot_hp != 0 && last_hero_hp == 0) {
        hero_revive_str.set_operate_type(OperateType::ADD);
      }
      else {
        hero_revive_str.set_operate_type(OperateType::DELETE);
      }
      last_hero_hp = pm02.game_robot_hp.blue_1_robot_hp;
    }

    //摩擦轮转速
    fric_speed_num.set_value(fric_target_speed);
    fric_speed_num.set_operate_type(OperateType::MODIFY);
    //电机存活状态
    chassis_dead_str.set_operate_type(chassis_alive ? OperateType::DELETE : OperateType::ADD);
    gimbal_dead_str.set_operate_type(gimbal_alive ? OperateType::DELETE : OperateType::ADD);
    shoot_dead_str.set_operate_type(shoot_alive ? OperateType::DELETE : OperateType::ADD);
    //动态UI
    if (dynamic_ui_num < 19) {
      dynamic_ui_num++;
    }
    else {
      dynamic_ui_num = 0;
    }
    //动态UI
    if (dynamic_ui_num == 1) {
      //电容条，总能量条
      ui_manager.pack(&cap_voltage, &energy_capacity);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 3) {
      ui_manager.pack(&projectile_allowance_num);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 5) {
      ui_manager.pack(&sentry_blood_num, &fric_speed_num);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 7) {
      ui_manager.pack(&supply_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 8) {
      ui_manager.pack(&shoot_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 9) {
      //框们
      ui_manager.pack(
        &fric_rect, &shoot_single_rect, &shoot_continue_rect, &follow_rect, &spin_rect,
        &autoaim_rect);
      pm02.send(ui_manager.data(), ui_manager.size());
    }
    if (dynamic_ui_num == 10) {
      ui_manager.pack(&hero_revive_str);
      pm02.send(ui_manager.data(), ui_manager.size());
    }
    if (dynamic_ui_num == 11) {
      ui_manager.pack(&autoaim_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 13) {
      ui_manager.pack(&autoaim_view_rect);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 15) {
      ui_manager.pack(&chassis_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 17) {
      ui_manager.pack(&gimbal_dead_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    if (dynamic_ui_num == 18) {
      ui_manager.pack(&autoaim_lost_str);
      pm02.send(ui_manager.data(), ui_manager.size());
      osDelay(33);
    }
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
  if (huart == &huart3) {
    remote.request();
  }
}