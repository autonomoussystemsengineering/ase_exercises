// ============================================================================
// Name        : mapping_with_known_poses.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
//               Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : The mapping exercise
// ============================================================================

// ROS
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Helper
#include <omp.h>
#include <boost/date_time/posix_time/posix_time.hpp>
using namespace std;

// Ros Listener Topic and publisher
string rosListenerTopicScan;
string rosListenerTopicOdom;
string rosPublisherTopic;
ros::Publisher pub;

// Map variables
static int dimensionX = 9;    // (m)
static int dimensionY = 9;    // (m)
static double mapResolution = 0.1; // (m)
static int mapDimensionX; // (cells)
static int mapDimensionY; // (cells)
static int mapEvaluationIter; // (1)
#define mattype uint16_t // CV_8U == uchar, CV_16U == short, CV_32FC1 == float
#define mattypecv CV_16U
#define maxValue 65535
static cv::Mat hit, miss; // (1)
static nav_msgs::OccupancyGrid ogm; // (P)

// Robot and sensor setup
static double robotOffsetX = dimensionX / 2.0f; // (m)
static double robotOffsetY = dimensionY / 2.0f; // (m)
static double maxReadingRange = 2.0f; // (m)
static double alpha = mapResolution; // (m/cell)
static double beta = 1 / 180.0 * M_PI; // (rad)

// program name
const string programName = "mapping_with_known_poses";

// Format the inner representation as OGM and send it
inline void showMap() {
  // Copy the probabilistic map data [0 .. 1] to the occupancy grid map format [0 .. 100]
  // Reference: http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
#pragma omp parallel for
  for (int yi = 0; yi < mapDimensionY; yi++) {
    for (int xi = 0; xi < mapDimensionX; xi++) {
      const char result = char((hit.at<mattype>(yi, xi) / (float) (hit.at<mattype>(yi, xi) + miss.at<mattype>(yi, xi))) * 100.0);
      ogm.data.at(yi * mapDimensionX + xi) = result;
    }
  }
  pub.publish(ogm);
}

void process(const sensor_msgs::LaserScan::ConstPtr &scan, const nav_msgs::Odometry::ConstPtr &odom) {
  static uint32_t iterator = 0;

  if (maxReadingRange >= scan->range_max) {
    ROS_WARN_STREAM_ONCE("max_reading_range is greater than the maximum range of the sensor, but should be lower (" << maxReadingRange << " >= " << scan->range_max << ")");
  }

  // allocate time variables for some statistics
  boost::posix_time::ptime before, after;
  boost::posix_time::time_duration msdiff;
  before = boost::posix_time::microsec_clock::local_time();

  // Store some usefull variables
  const int rangeSize = scan->ranges.size();
  const float scanAngleMin = scan->angle_min;
  const float scanAngleIncrement = scan->angle_increment;

  // Get the rotations of the robot
  double roll, pitch, yaw;
  tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  // ###  START PROGRAMMING ###
  // Store the position (xx, xy) and orientation (xt) of the robot in the map
  // const double xx =
  // const double xy =
  // const double xt =



  // ###  END PROGRAMMING   ###

  // Update every cell regarding the scan
#pragma omp parallel for
  for (int yi = 0; yi < mapDimensionY; yi++) {
    for (int xi = 0; xi < mapDimensionX; xi++) {
      // ###  START PROGRAMMING ### (approx. 20 lines of code)
      // Hint 1: Calculate the robot's position as cells in the map
      // Hint 2: Make use of the atan2 function for calculating the orientation of a scan with respect to the robot's pose in the world










      // Hint: Think about the conditions, when to update which cell
      if (true /* replace true with proper condition */) {
        // Do nothing if the cell is not inside the scan (this section remains empty)
      } else if (true /* replace true with proper condition */) {
        // Update the hit map cell
        // hit.at<mattype>(yi, xi) = 
      } else if (true /* replace true with proper condition */) {
        // Update the miss map cell
        // miss.at<mattype>(yi, xi) = 
      }












      // ###  END PROGRAMMING   ###
    }
  }

  // Print out some information and send the map every mapEvaluationIter'th iteration
  after = boost::posix_time::microsec_clock::local_time();
  msdiff = after - before;
  ROS_INFO_STREAM("time for map update: " << msdiff.total_milliseconds() << "ms");
  if (!(++iterator % mapEvaluationIter)) {
    ROS_INFO("-- Publish map --");
    ogm.header.stamp = scan->header.stamp;
    showMap();
  }

}

int main(int argc, char *argv[]) {

  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  // Handle parameters
  node.param<string>("ros_listener_topic_scan", rosListenerTopicScan, "/amiro1/laser");
  node.param<string>("ros_listener_topic_odom", rosListenerTopicOdom, "/amiro1/odom");
  node.param<string>("ros_publisher_topic", rosPublisherTopic, "/map");
  node.param<int>("dimension_x", dimensionX, 9);
  node.param<int>("dimension_y", dimensionY, 9);
  node.param<double>("map_resolution", mapResolution, 0.1);
  node.param<double>("max_reading_range", maxReadingRange, 2.0);
  node.param<int>("map_evaluation_iter", mapEvaluationIter, 20);

  // Print some information
  ROS_INFO("ros_listener_topic_scan: %s", rosListenerTopicScan.c_str());
  ROS_INFO("ros_listener_topic_odom: %s", rosListenerTopicOdom.c_str());
  ROS_INFO("ros_publisher_topic_odom: %s", rosPublisherTopic.c_str());

  // Robot and sensor setup (center the robot position in the world)
  robotOffsetX = dimensionX / 2.0f;
  robotOffsetY = dimensionY / 2.0f;
  alpha = mapResolution;
  beta = 1 / 180.0 * M_PI;

  // Publisher: Send the inter map representation
  pub = node.advertise<nav_msgs::OccupancyGrid>(rosPublisherTopic, 1);

  // Subscriber: Sync the scan and odom topic
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(node, rosListenerTopicScan, 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node, rosListenerTopicOdom, 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), scan_sub, odom_sub);
  sync.registerCallback(boost::bind(&process, _1, _2));

  // Allocate the occupancy grid and center the position in the world
  mapDimensionX = int(dimensionX / mapResolution) + 1;
  mapDimensionY = int(dimensionY / mapResolution) + 1;
  ogm.header.frame_id = "world";
  ogm.info.resolution = mapResolution;
  ogm.info.width = mapDimensionX;
  ogm.info.height = mapDimensionY;
  ogm.info.origin.position.x = -dimensionX / 2.0f;
  ogm.info.origin.position.y = -dimensionY / 2.0f;
  ogm.data.resize(ogm.info.width * ogm.info.height, -1);
  hit = cv::Mat(mapDimensionY, mapDimensionX, mattypecv, cv::Scalar(1));
  miss = cv::Mat(mapDimensionY, mapDimensionX, mattypecv, cv::Scalar(1));

  // Execute the mapping
  ros::spin();

  return 0;
}
