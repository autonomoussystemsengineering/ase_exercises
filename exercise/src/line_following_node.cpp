// ============================================================================
// Name        : line_following_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : This node handle the line following behavior for the AMiRo
// ============================================================================

// ROS
#include <ros/ros.h>

#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

using namespace std;

static double kp, kd, ki;
static int LINE_THRESH_BLACK, LINE_THRESH_ROT_BLACK;

enum state {
  linefollowing,
  rotation,
  waitforcommand
};

// params for smoothingX forward velocity
int smoothing_counterX = 0;
const int smoothing_histX = 10;
double smoothing_vecX[smoothing_histX];

struct floorSensorBuff {
  static const int _max = 4;
  uint values[_max];
  int pos = 0;

  void push_back(uint value) {
    values[pos] = value;
    pos++;
    pos = pos % _max;
  };

  double average() {
    double sum = 0.0;
    for (int i = 0; i < _max; i++) {
      sum += values[i];
    }
    return sum / (double) _max;
  };
};


// forward velocity
static int command = INT8_MAX;
static double vel_x = 0.1;

// Twist Publisher
static ros::Publisher twist_pub;

// current State of line_following-STM
static state currentState;

//current Sensors
static uint floorSensor[4];
struct floorSensorBuff floorSensor0, floorSensor3;
static bool floor1recentlyBlack = false;
static bool floor03recentlyBlack = false;


void floorSubCallback(const amiro_msgs::UInt16MultiArrayStamped::ConstPtr floor) {
  if (floor->array.data.size() == 4)
    for (uint i = 0; i < floor->array.data.size(); i++)
      floorSensor[i] = floor->array.data[i];
}

void commandSubCallback(const std_msgs::Int8::ConstPtr cmd) {
  command = cmd->data;
}

// Smoothing forward velocity
double smoothingX(const double value) {
  double sum = 0.0;
  smoothing_vecX[smoothing_counterX] = value;
  smoothing_counterX++;
  smoothing_counterX = smoothing_counterX % smoothing_histX;

  for (int i = 0; i < smoothing_histX; i++) {
    sum += smoothing_vecX[i];
  }
  return std::max(sum / (double) smoothing_histX, 0.0);
}

geometry_msgs::Twist lineFollowingFunction() {
  geometry_msgs::Twist twist;

  static double last_error;
  static double integral;

  // if outer right sensors notice black
  if (floorSensor[1] < (uint) LINE_THRESH_BLACK && !floor1recentlyBlack) {
    ROS_INFO("[%s] Detected outer line. Stopping robot.", ros::this_node::getName().c_str());
    floor1recentlyBlack = true;
    currentState = state::waitforcommand;
    return twist;
  } else if (floorSensor[1] > (uint) LINE_THRESH_BLACK) {
    floor1recentlyBlack = false;
  }
  // Lazy Linefollowing:
  //white 30k-45k
  //black 5k-7k
  floorSensor0.push_back(floorSensor[0]);
  floorSensor3.push_back(floorSensor[3]);
  double error = floorSensor0.average() - floorSensor3.average();
  double value = (kp * error + kd * (error - last_error) + ki * integral);
  twist.angular.z = 1e-5 * value;
  ROS_DEBUG("value: %f \t twist.angular.z: %f", value, twist.angular.z);
  ROS_DEBUG("kp: %f %f, kd: %f %f: ki: %f %f", error, ki * error, error - last_error, kd * (error - last_error), integral, ki * integral);
//  if (twist.angular.z / abs(twist.angular.z) != angularSign)
//    twist.angular.z = twist.angular.z * 0.1;
//  angularSign = twist.angular.z / abs(twist.angular.z);
//  twist.angular.z = std::min(twist.angular.z, M_PI_2);
//  twist.angular.z = std::max(twist.angular.z, -M_PI_2);
  twist.linear.x = smoothingX(std::max(0.02, vel_x - 0.25 * fabs(twist.angular.z)));
//  twist.linear.x = vel_x;

  integral += error;
  last_error = error;

  return twist;
}

geometry_msgs::Twist rotationFunction() {
  geometry_msgs::Twist twist;

  // check if end rotation->command to INT8_MAX
  if (floorSensor[0] < (uint) LINE_THRESH_ROT_BLACK && floorSensor[3] < (uint) LINE_THRESH_ROT_BLACK && !floor03recentlyBlack) {
    currentState = state::linefollowing;
    command = INT8_MAX;
    return twist;
  } else if (floorSensor[0] > (uint) LINE_THRESH_ROT_BLACK && floorSensor[3] > (uint) LINE_THRESH_ROT_BLACK) {
    floor03recentlyBlack = false;
  }

  twist.angular.z = command * 0.7;

  return twist;
}

void mainLoop() {
  geometry_msgs::Twist twist;

  switch (currentState) {
    case state::linefollowing:
      twist = lineFollowingFunction();
      break;
    case state::rotation:
      twist = rotationFunction();
      break;
    case state::waitforcommand:
      switch (command) {
        case -1:
        case 1:
          ROS_INFO("[%s] Switched to rotation with command: %d.", ros::this_node::getName().c_str(), command);
          currentState = state::rotation;
          floor03recentlyBlack = true;
          break;
        case 0:
          ROS_INFO("[%s] Switched to line_following with command: %d.", ros::this_node::getName().c_str(), command);
          command = INT8_MAX;
          currentState = state::linefollowing;
          break;
        case INT8_MAX:
          ROS_INFO_THROTTLE(2, "[%s] Wait for command.", ros::this_node::getName().c_str());
          break;
        default:
          command = std::max(command, -1);
          command = std::min(command, 1);
      }
      smoothingX(twist.linear.x);
      break;
  }

  twist_pub.publish(twist);
}

int main(int argc, char *argv[]) {

  // Init ROS
  ros::init(argc, argv, ros::this_node::getName().c_str());
  ros::NodeHandle node("~");

  int amiroId;

  node.param("amiro_id", amiroId, 1);
  node.param("vel_x", vel_x, 0.1);
  node.param("kp", kp, 2.1);
  node.param("ki", ki, 0.2);
  node.param("kd", kd, 5.0);
  node.param("LINE_THRESH_BLACK", LINE_THRESH_BLACK, 10000);
  node.param("LINE_THRESH_ROT_BLACK", LINE_THRESH_ROT_BLACK, 7500);

  ros::Subscriber cmd_sub = node.subscribe("/amiro" + to_string(amiroId) + "/cmd_lf", 1, commandSubCallback);
  ros::Subscriber floor_sub = node.subscribe("/amiro" + to_string(amiroId) + "/proximity_floor/values", 1, floorSubCallback);
  twist_pub = node.advertise<geometry_msgs::Twist>("/amiro" + to_string(amiroId) + "/cmd_vel", 1);

  ros::spinOnce();
  sleep(1);

  currentState = state::linefollowing;
  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    mainLoop();
    r.sleep();
  }

  return 0;
}
