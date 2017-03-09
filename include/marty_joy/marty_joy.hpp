/**
 * @file      marty_joy.hpp
 * @brief     Marty is controlled remotely using a joystick game controller
 * @author    Alejandro Bordallo <alex.bordallo@robotical.io>
 * @date      2017-03-08
 * @copyright (Apache) 2017 Robotical Ltd.
 */

#ifndef BALL_FOLLOWER_HPP
#define BALL_FOLLOWER_HPP

// ROS
#include <ros/ros.h>

// MESSAGES
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <marty_msgs/Command.h>
#include <marty_msgs/ServoMsgArray.h>

#define EYES_ANGRY   30
#define EYES_NORMAL  0
#define EYES_EXCITED -40
#define EYES_WIDE  -120

enum Request {
  NO_REQ = 0,
  L_KICK,
  R_KICK,
  CELEB,
  L_SIDE,
  R_SIDE,
  L_LEAN,
  R_LEAN,
  F_LEAN,
  B_LEAN,
  STRAIGHT
};

class MartyJoy {
 protected:
  ros::NodeHandle nh_;

  void loadParams();
  void init();
  void rosSetup();

 public:
  MartyJoy(ros::NodeHandle& nh);
  ~MartyJoy();

  // void stop();

 private:
  void joyCB(const sensor_msgs::Joy& msg);
  void acCB(const ros::TimerEvent& e);

  // Flags
  bool joy_msg_;
  bool enabled_;

  int move_time_;
  float refresh_time_;
  int forw_amount_;
  // int left_amount_;
  int turn_amount_;
  int arm_amount_;

  int request_;
  int eye_pos_;

  std::vector<float> axes_;
  std::vector<int> buttons_;

  // ROS
  ros::Subscriber joy_sub_;
  ros::Publisher enable_pub_;
  ros::Publisher servo_pub_;
  ros::ServiceClient command_srv_;
  ros::Timer action_timer_;
  marty_msgs::Command stop_;
  marty_msgs::Command walk_;
  marty_msgs::Command side_step_;
  marty_msgs::Command kick_;
  marty_msgs::Command celeb_;
  marty_msgs::Command lean_;
  marty_msgs::Command eyes_;
  marty_msgs::Command straight_;
  marty_msgs::ServoMsg left_arm_;
  marty_msgs::ServoMsg right_arm_;
};

#endif  /* BALL_FOLLOWER_HPP */
