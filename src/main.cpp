#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <GPSConversion.h>

using namespace std;

// MASTER DATA
ros::Publisher pub_gpsref, pub_hist;
unsigned int poses_seq = 0;
bool is_first = true;
bool save_file = false;
std::ofstream outfile;
sensor_msgs::NavSatFix gpsref;
vector<geometry_msgs::PoseStamped> poses;
tf::TransformBroadcaster *mTfBr;

// MASTER FUNCTIONS
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

/**
 * Main function
 * Sets up the listeners and publishers needed
 */
int main(int argc, char **argv) {

  // Debug message
  ROS_INFO("Starting up");

  // Startup this node
  ros::init(argc, argv, "gps_path_pub");

  // Setup the listeners here
  ros::NodeHandle nh("~");
  ros::Subscriber sub_gps;

  std::string file_path;
  nh.param<bool>("save_file", save_file, false);
  nh.param<std::string>("file_path", file_path, "");
  if (save_file) {

    // Create folder path to this location if not exists
    boost::filesystem::path dir(file_path.c_str());
    if (boost::filesystem::create_directories(dir.parent_path())) {
      ROS_INFO("Created folder path to output file.");
      ROS_INFO("Path: %s", dir.parent_path().c_str());
    }
    // If it exists, then delete it
    if (boost::filesystem::exists(file_path)) {
      ROS_WARN("Output file exists, deleting old file....");
      boost::filesystem::remove(file_path);
    }

    // Open this file we want to write to
    outfile.open(file_path.c_str());
    if (outfile.fail()) {
      ROS_ERROR("Unable to open output file!!");
      ROS_ERROR("Path: %s", file_path.c_str());
      std::exit(EXIT_FAILURE);
    }
    outfile << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 "
               "Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33"
            << std::endl;
  }

  sub_gps = nh.subscribe("/mti0/sensor/fix_navsat", 10, gpsCallback);
  ROS_INFO("Subscribed: %s", sub_gps.getTopic().c_str());

  // Setup the publishers here
  pub_gpsref = nh.advertise<sensor_msgs::NavSatFix>("/gps_path_pub/gpsref", 2);
  ROS_INFO("Publishing: %s", pub_gpsref.getTopic().c_str());
  pub_hist = nh.advertise<nav_msgs::Path>("/gps_path_pub/line", 2);
  ROS_INFO("Publishing: %s", pub_hist.getTopic().c_str());

  // Setup our transform broadcaster
  mTfBr = new tf::TransformBroadcaster();

  // Debug
  ROS_INFO("Ready. Waiting for messages...");

  // ROS spin
  ros::spin();

  // We are done, so return
  ROS_INFO("DONE EXITING");
  return EXIT_SUCCESS;
}

/**
 * Callback from the GPS sensor
 * Will calculate the ENU and then publish the path and transforms needed
 */
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {

  // Check that we have a value GPS message from the system
  // http://docs.ros.org/jade/api/sensor_msgs/html/msg/NavSatStatus.html
  // if(msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
  //    return;
  //}

  // Save datum
  if (is_first) {
    gpsref = *msg;
    is_first = false;
    return;
  }

  // Convert into local frame
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";
  GPSConversion::GeodeticToEnu((*msg).latitude, (*msg).longitude, (*msg).altitude, gpsref.latitude, gpsref.longitude, gpsref.altitude,
                               pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  // Append to file if we have it
  if (save_file) {

    // timestamp
    outfile.precision(5);
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile << msg->header.stamp.toSec() << " ";

    // pose
    outfile.precision(6);
    outfile << pose.pose.position.x << " " << pose.pose.position.y << " " << pose.pose.position.z;
    outfile << " " << 0 << " " << 0 << " " << 0 << " " << 1;

    // covariance ori then pos
    outfile.precision(10);
    outfile << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0;
    outfile << " " << msg->position_covariance[0] << " " << msg->position_covariance[1] << " " << msg->position_covariance[2] << " "
            << msg->position_covariance[4] << " " << msg->position_covariance[5] << " " << msg->position_covariance[8] << std::endl;
  }

  // Append to our array
  poses.push_back(pose);

  // Create our path object
  nav_msgs::Path arrGps;
  arrGps.header.stamp = ros::Time::now();
  arrGps.header.seq = poses_seq;
  arrGps.header.frame_id = "map";
  arrGps.poses = poses;

  // Publish our transform
  tf::StampedTransform tfLinM;
  tfLinM.stamp_ = ros::Time::now();
  tfLinM.frame_id_ = "map";
  tfLinM.child_frame_id_ = "gps";
  tf::Quaternion quat(0, 0, 0, 1);
  tfLinM.setRotation(quat);
  tf::Vector3 orig(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  tfLinM.setOrigin(orig);
  mTfBr->sendTransform(tfLinM);

  // Publish our reference and line
  pub_gpsref.publish(gpsref);
  pub_hist.publish(arrGps);

  // Move forward
  poses_seq++;

  // Printout every so often
  if (poses_seq % 100 == 0) {
    ROS_INFO("Published %d GPS path messages", poses_seq);
  }
}
