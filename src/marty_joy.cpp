/**
 * @file      marty_joy.cpp
 * @brief     Marty is controlled remotely using a joystick game controller
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2017-03-08
 * @copyright (Apache) 2017 Robotical Ltd.
 */

#include <marty_joy/marty_joy.hpp>

MartyJoy::MartyJoy(ros::NodeHandle& nh) : nh_(nh) {
  this->loadParams();
  this->init();
  this->rosSetup();
  ROS_INFO("MartyJoy Ready!");
}

MartyJoy::~MartyJoy() {
  command_srv_.call(stop_);
}

void MartyJoy::loadParams() {
  nh_.param("LARM/zero", larm_z_, 0);
  nh_.param("RARM/zero", rarm_z_, 0);
  move_time_ = 2000;
  refresh_time_ = 2.0;
  forw_amount_ = 50;
  turn_amount_ = 20;
  arm_amount_ = 60;
}

void MartyJoy::init() {
  request_ = NO_REQ;
  enabled_ = false;
  enable_ready_ = true;
  called_ = false;
  eye_pos_ = EYES_NORMAL;
  stop_.request.data.push_back(stop_.request.CMD_STOP);

  walk_.request.data.push_back(walk_.request.CMD_WALK);
  walk_.request.data.push_back(1);            //  Steps
  walk_.request.data.push_back(turn_amount_); //  Turn
  walk_.request.data.push_back(move_time_);   //  MoveTime
  walk_.request.data.push_back(0);            //  StepLength
  walk_.request.data.push_back(-1);           //  Side

  side_step_.request.data.push_back(side_step_.request.CMD_SIDESTEP);
  side_step_.request.data.push_back(0);      //  CMD_LEFT = 0
  side_step_.request.data.push_back(1);      //  Steps
  side_step_.request.data.push_back(move_time_); //  MoveTime
  side_step_.request.data.push_back(50);     //  StepLength

  kick_.request.data.push_back(kick_.request.CMD_KICK);
  kick_.request.data.push_back(1);           //  CMD_LEFT = 0
  kick_.request.data.push_back(move_time_);  //  MoveTime

  celeb_.request.data.push_back(celeb_.request.CMD_CELEBRATE);
  celeb_.request.data.push_back(move_time_ * 2); //  MoveTime

  lean_.request.data.push_back(lean_.request.CMD_LEAN);
  lean_.request.data.push_back(0);          //  Direction
  lean_.request.data.push_back(40);         //  Amount
  lean_.request.data.push_back(move_time_); //  MoveTime

  eyes_.request.data.push_back(eyes_.request.CMD_EYES);
  eyes_.request.data.push_back(eye_pos_);          //  Amount

  straight_.request.data.push_back(straight_.request.CMD_STRAIGHT);
  straight_.request.data.push_back(move_time_);

  left_arm_.servo_id = 6;
  left_arm_.servo_cmd = 0;
  right_arm_.servo_id = 7;
  right_arm_.servo_cmd = 0;
}

void MartyJoy::rosSetup() {
  joy_sub_ = nh_.subscribe("joy", 1000, &MartyJoy::joyCB, this);
  ros::service::waitForService("/marty/command");
  command_srv_ = nh_.serviceClient<marty_msgs::Command>("command", true);
  action_timer_ = nh_.createTimer(ros::Duration(refresh_time_),
                                  &MartyJoy::acCB, this, true, false);
  enable_pub_ = nh_.advertise<std_msgs::Bool>("enable_motors", 10);
  servo_pub_ = nh_.advertise<marty_msgs::ServoMsgArray>("servo_array", 10);
}

void MartyJoy::joyCB(const sensor_msgs::Joy& msg) {
  axes_ = msg.axes;
  buttons_ = msg.buttons;
  // axes_[0] // SideLeft/Right 1/-1
  // axes_[1] // Forward/Backward 1/-1
  // axes_[2] // LeftArm un/pressed 1/-1
  // axes_[3] // TurnLeft/Right 1/-1
  // axes_[5] // RightArm un/pressed 1/-1
  // buttons_[0] // A Button
  // buttons_[1] // B Button
  // buttons_[2] // X Button
  // buttons_[3] // Y Button
  // buttons_[7] // Start Button

  if (buttons_[7] == 0) { enable_ready_ = true; }

  // MOVE REQUESTS
  if ((buttons_[7] == 1) && (enable_ready_ == true)) {   // Start Button
    enabled_ = !enabled_;
    enable_ready_ = false;
    if (enabled_) { ROS_INFO("ENABLING!"); }
    else { ROS_INFO("DISABLING!"); }
    std_msgs::Bool enable;
    enable.data = enabled_;
    enable_pub_.publish(enable);
    // if (enabled_) { command_srv_.call(straight_); }
  }
  if (enabled_) {
    if (buttons_[0] == 1) {   // A Button
      ROS_INFO("EYES!");
      if (eye_pos_ == EYES_NORMAL) {
        eye_pos_ = EYES_EXCITED;
      } else if (eye_pos_ == EYES_EXCITED) {
        eye_pos_ = EYES_WIDE;
      } else if (eye_pos_ == EYES_WIDE) {
        eye_pos_ = EYES_ANGRY;
      } else if (eye_pos_ == EYES_ANGRY) {
        eye_pos_ = EYES_NORMAL;
      }
      eyes_.request.data[1] = eye_pos_;
      command_srv_.call(eyes_);
    } else if (buttons_[3] == 1) {
      request_ = CELEB;
    } else if (buttons_[4] == 1) {
      request_ = L_KICK;
    } else if (buttons_[5] == 1) {
      request_ = R_KICK;
    } else if (axes_[0] > 0.5) {
      request_ = L_SIDE;
    } else if (axes_[0] < -0.5) {
      request_ = R_SIDE;
    } else if (axes_[6] == 1) {
      request_ = L_LEAN;
    } else if (axes_[6] == -1) {
      request_ = R_LEAN;
    } else if (axes_[7] == 1) {
      request_ = F_LEAN;
    } else if (axes_[7] == -1) {
      request_ = B_LEAN;
    }
    // Walk if no other move was requested
    if ((request_ == NO_REQ) && ((axes_[3] != 0.0) || (axes_[1] != 0.0))) {
      request_ = WALK;
    }
    if (!called_) {
      if (request_ == NO_REQ) {
        // ANALOGUE ARM CONTROL
        left_arm_.servo_cmd = arm_amount_ + (-arm_amount_ * axes_[2] + larm_z_);
        right_arm_.servo_cmd = -arm_amount_ + (arm_amount_ * axes_[5] + rarm_z_);
        left_arm_.servo_cmd = std::max(-126, (int)left_arm_.servo_cmd);
        right_arm_.servo_cmd = std::max(-126, (int)right_arm_.servo_cmd);
        left_arm_.servo_cmd = std::min(126, (int)left_arm_.servo_cmd);
        right_arm_.servo_cmd = std::min(126, (int)right_arm_.servo_cmd);

        marty_msgs::ServoMsgArray servo_msg_array;
        servo_msg_array.servo_msg.push_back(left_arm_);
        servo_msg_array.servo_msg.push_back(right_arm_);

        servo_pub_.publish(servo_msg_array);
      } else {
        called_ = true;
        this->act();
      }
    }
  }
}

void MartyJoy::acCB(const ros::TimerEvent& e) {
  called_ = false;
  request_ = NO_REQ;
  action_timer_.stop();
}

void MartyJoy::act() {
  if (request_ == WALK) {
    walk_.request.data[2] = turn_amount_ * -axes_[3];
    walk_.request.data[4] = forw_amount_ * axes_[1];
    command_srv_.call(walk_);
  } else if (request_ == L_KICK) {
    kick_.request.data[1] = 0;
    command_srv_.call(kick_);
  } else if (request_ == R_KICK) {
    kick_.request.data[1] = 1;
    command_srv_.call(kick_);
  } else if (request_ == CELEB) {
    command_srv_.call(celeb_);
  } else if (request_ == L_SIDE) {
    side_step_.request.data[1] = 0;
    command_srv_.call(side_step_);
  } else if (request_ == R_SIDE) {
    side_step_.request.data[1] = 1;
    command_srv_.call(side_step_);
  } else if (request_ == L_LEAN) {
    lean_.request.data[1] = 0;
    command_srv_.call(lean_);
  } else if (request_ == R_LEAN) {
    lean_.request.data[1] = 1;
    command_srv_.call(lean_);
  } else if (request_ == F_LEAN) {
    lean_.request.data[1] = 2;
    command_srv_.call(lean_);
  } else if (request_ == B_LEAN) {
    lean_.request.data[1] = 3;
    command_srv_.call(lean_);
  }
  action_timer_.start();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "marty_joy");
  ros::NodeHandle nh("");
  MartyJoy marty_joy(nh);

  ros::Rate r(20);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
