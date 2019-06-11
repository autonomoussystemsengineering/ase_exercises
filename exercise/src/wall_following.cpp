// ============================================================================
// Name        : wall_following.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
//               Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : The wall following exercise
// ============================================================================

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

// ROS listener and subscriber setup
string rosListenerTopic;
string rosPublisherTopic;
ros::Publisher pub;

// program name
const string programName = "wall_following";




// Calculate degree to radiant
float degreeToRad(const float degree);

// Process the scan and send the twist message
void process(const sensor_msgs::LaserScan::ConstPtr &scan);

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("ros_listener_topic", rosListenerTopic, "/input");
  node.param<string>("ros_publisher_topic", rosPublisherTopic, "/output");


  ROS_INFO("ros_listener_topic: %s", rosListenerTopic.c_str());
  ROS_INFO("ros_publisher_topic: %s", rosPublisherTopic.c_str());

  ros::Subscriber sub = node.subscribe(rosListenerTopic, 1, process);
  pub = node.advertise<geometry_msgs::Twist>(rosPublisherTopic, 1);
  ros::spin();

  return 0;
}

void process(const sensor_msgs::LaserScan::ConstPtr &scan) {
  geometry_msgs::Twist twist;

  int min_scan_index = -1;
  float min_scan_value = scan->ranges[0];

  int max = (scan->ranges.size() / 8) * 3;

  // Analyze the sensor scan (scan->ranges[i])
  for (int i = 0; i < max; i++) {
    if (scan->ranges[i] <= min_scan_value) {
      min_scan_value = scan->ranges[i];
      min_scan_index = i;
    }
  }
  ROS_INFO("min scan: %d with value: %f", min_scan_index, min_scan_value);

  // ###  START PROGRAMMING ###


















  // ###  END PROGRAMMING   ###

  pub.publish(twist);
}

float degreeToRad(const float degree) {
  return (float) (degree * M_PI / 180.0f);
}
