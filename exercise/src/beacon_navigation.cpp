// ============================================================================
// Name        : beacon_navigation.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
//               Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : The beacon navigation exercise
// ============================================================================

// ROS
#include <ros/ros.h>
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Int8.h"
// Helper
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
using namespace boost;

// The graph structure and definition the directed graph G by its edges E
typedef adjacency_list<listS, vecS, directedS, no_property,
    property<edge_weight_t, int> > Graph;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef std::pair<int, int> E;
const int num_nodes = 9;
E map_edges[] =
    { E(100, 102), E(100, 101),
      E(101, 102), E(102, 20),
      E(20, 21), E(20, 22),
      E(21, 22),
      E(22, 10),
      E(10, 11), E(10, 12),
      E(11, 12),
      E(12, 100) };
int weights[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  // Set constant weights to all edges
Graph G(map_edges, map_edges + sizeof(map_edges) / sizeof(E), weights,
        num_nodes);

// State machine configuration
enum states {
  INIT,
  DRIVE,
  DECIDE,
  EXIT,
  FETCH
};
std::string state2str[] = { "INIT", "DRIVE", "DECIDE", "EXIT", "FETCH" };

// Get the reversed node list between start and end id
std::vector<int> getShortestPath(int id_start, int id_goal);

// Stores the current RFID tag
void callback(const std_msgs::UInt32MultiArray msg);
unsigned int rfidTagId = 0;

// program name
const std::string programName = "beacon_navigation";

int main(int argc, char *argv[]) {

  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  // Handle parameters
  std::string rosListenerTopicRfid, rosPublisherTopicCmdlf;
  int goalId, smRate;
  node.param<std::string>("sub_topic_rfid", rosListenerTopicRfid, "/amiro1/rfid_tag_list");
  node.param<std::string>("pub_topic_cmdlf", rosPublisherTopicCmdlf, "/amiro1/cmd_lf");
  node.param<int>("goal_id", goalId, -1);  // Goal id
  node.param<int>("sm_rate", smRate, 10);  // The clock for the state machine

  // Print some information
  ROS_INFO("sub_topic_rfid: %s", rosListenerTopicRfid.c_str());
  ROS_INFO("pub_topic_cmdlf: %s", rosPublisherTopicCmdlf.c_str());
  ROS_INFO("goal_id: %d", goalId);
  ROS_INFO("sm_rate: %d", smRate);
  if (goalId < 0) {
    ROS_ERROR("Choose a valid goal node via parameter ~/goal_id");
    node.shutdown();
    return -1;
  }

  // This is just an example to find the path between id 11 and 102
  std::stringstream path_string;
  int id_start = 11;
  int id_goal = 102;
  path_string << "TEST: path from goal (" << id_goal << ") to start ("
              << id_start << "): ";
  std::vector<int> node_list_test = getShortestPath(id_start, id_goal);
  for (auto it = node_list_test.begin(); it != node_list_test.end(); ++it) {
    path_string << *it << " ";
  }
  ROS_INFO_STREAM(path_string.str());

  // Init the subscriber and publisher
  ros::Subscriber sub = node.subscribe(rosListenerTopicRfid, 1, callback);
  ros::Publisher pub = node.advertise<std_msgs::Int8>(rosPublisherTopicCmdlf, 1, true);

  // Execute the state machine
  std::vector<int> node_list;  // List of beacon ids to drive to
  int startId = -1;            // The beacon id where the robot is located in INIT
  int lastId = -1;             // Last visited beacon id
  int currentId = -1;          // Current visiting beacon id
  int nextId = -1;             // Next beacon id to visit
  states state = INIT;         // The current state of the state machine
  states nextState = INIT;     // The next state of the state machine
  std_msgs::Int8 cmdLf;        // Drive commands are 1:right-straight, 0:straight, -1:left-straight
  int pathCounter = 1;         // The path counter
  int smCounter = 0;           // Just a counter for printing information once a second
  ros::Rate rate(smRate);      // The rate of the state machine
  while (ros::ok() && node.ok()) {
    // Get the current RFID sensor data
    ros::spinOnce();
    // State machine
    switch (state) {
      case INIT:
        // ###  START PROGRAMMING ###
        if (rfidTagId != 0) {
          // 1. Fetch the current position 'startId' and set 'lastId = currentId = startId'



          // 2. Find the path from 'startId' to 'goalId' and store it into 'node_list'

          // ###  END PROGRAMMING   ###
          nextState = DECIDE;
        } else {
          nextState = INIT;
        }
        break;
      case DRIVE:
        // ###  START PROGRAMMING ###
        // Execute DRIVE as long as we see the last RFID, otherwise go to the DECIDE state







        // ###  END PROGRAMMING   ###
        break;
      case DECIDE:
        // ###  START PROGRAMMING ###
        // If the current id is the goal id, go to the exit state, otherwise continue in the sm





        // ###  END PROGRAMMING   ###
        break;
      case FETCH:
        // ###  START PROGRAMMING ###
        // 1. Fetch the next id to drive to and store the command to 'cmdLf.data'
        // Assume that we always stand in a station in the beginning
        // All cases can be derived by using the modulo operator: e.g: nextId % 10 == 2 ==> Go straight








        // 2. Execute the steering command by publishing 'cmdLf' via 'pub.publish()'

        // ###  END PROGRAMMING   ###
        nextState = DRIVE;
        break;
      case EXIT:
        node.shutdown();
        break;
    }
    // Print information once a second or if the state switches
    if (++smCounter % smRate == 0 || state != nextState) {
      ROS_INFO_STREAM("----- SM INFO START -----");
      ROS_INFO_STREAM("current state -> next state: " << state2str[state] << " -> " << state2str[nextState]);
      ROS_INFO_STREAM("start id, goal id: " << startId << ", " << goalId);
      ROS_INFO_STREAM("last id, current id, next id: " << lastId << ", " << currentId << ", " << nextId);
      std::stringstream path_string;
      path_string << "path from goal (" << goalId << ") to start (" << startId << "): ";
      for (auto it = node_list.begin(); it != node_list.end(); ++it) { path_string << *it << " ";}
      ROS_INFO_STREAM(path_string.str());
      ROS_INFO_STREAM("----- SM INFO END   -----");
    }
    state = nextState;
    rate.sleep();
  }

  return 0;
}

std::vector<int> getShortestPath(int id_start, int id_goal) {

  // vector for storing distance property
  std::vector<int> d(num_vertices(G));

  // get the start and goal vertex
  Vertex start = vertex(id_start, G);
  Vertex goal = vertex(id_goal, G);
  // invoke Dijkstra's algorithm
  std::vector<Vertex> p(num_vertices(G));
  dijkstra_shortest_paths(
      G,
      start,
      predecessor_map(
          boost::make_iterator_property_map(p.begin(),
                                            get(boost::vertex_index, G)))
          .distance_map(
          boost::make_iterator_property_map(d.begin(),
                                            get(boost::vertex_index, G))));

  // Print out the distance map between the start vertex and all others
  std::stringstream distances_string;
  distances_string << "\ndistances from start vertex:\n";
  graph_traits<Graph>::vertex_iterator vi;
  int idx = 0;
  for (vi = vertices(G).first; vi != vertices(G).second; ++vi)
    distances_string << "distance(" << idx++ << ") = " << d[*vi] << "\n";
  ROS_DEBUG_STREAM(distances_string.str());

  // Get the shortest path
  std::vector<Vertex> path;
  Vertex current = goal;

  while (current != start) {
    path.push_back(current);
    current = p[current];
  }
  path.push_back(start);

  //This prints the path reversed use reverse_iterator and rbegin/rend
  std::vector<int> node_list;
  std::stringstream path_string;
  path_string << "path from goal (" << id_goal << ") to start (" << id_start
              << "): ";
  for (auto it = path.begin(); it != path.end(); ++it) {
    path_string << *it << " ";
    node_list.push_back(*it);
  }
  ROS_DEBUG_STREAM(path_string.str());

  return node_list;
}

void callback(const std_msgs::UInt32MultiArray msg) {
  if (msg.data.size() > 1) {
    ROS_WARN("Got multiple RFID tags. Won't store any of these.");
  } else if (msg.data.size() == 1) {
    rfidTagId = *msg.data.begin();
    ROS_DEBUG_STREAM("Got RFID tag " << rfidTagId);
  }
}
