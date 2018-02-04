#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
// For rand() and RAND_MAX

struct
{
  float min;
  float max;
  float inc;
  std::vector<float> ranges;
} g_laser;

const float FRONT_DEGREE{ 0.0 };
const float G_CONVERSION_RATE{ 57.2957795 };
const float THRESHOLD{ 3.0 };

void processLaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int n_ranges = msg->ranges.size();
  double nearest = *(std::min_element(msg->ranges.begin(), msg->ranges.end()));
  double farest = *(std::max_element(msg->ranges.begin(), msg->ranges.end()));
  //  ROS_INFO("[robot_explorer] I've a total of %i measurements to process! Are you ready? Nearest=%.2f Far=%.2f",
  //           n_ranges, nearest, farest);
  // Cutting fov for accuracy
  float min, max, inc;
  min = (msg->angle_min + 120 * msg->angle_increment);  //
  max = (msg->angle_min + 840 * msg->angle_increment);  //
  inc = msg->angle_increment;                           // 0.25l

  //  ROS_INFO("[robot_explorer] Shorting fov");
  auto beg = msg->ranges.begin();
  auto end = msg->ranges.begin();
  std::advance(beg, 120);
  std::advance(end, 840);
  std::vector<float> ranges(beg, end);
  g_laser.min = min;
  g_laser.max = max;
  g_laser.inc = inc;
  g_laser.ranges = ranges;
  // Send a message to rosout with the details .
  //  ROS_INFO("[robot_explorer] min=%.2f, angle_max=%.2f, angle_inc=%.2f", min, max, g_laser.inc);
};
void processOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // auto pose = msg->pose;
}

float distanceToObsAt(float degree)
{
  degree /= G_CONVERSION_RATE;
  int i = degree / g_laser.inc;

  return g_laser.ranges.at(i);
}

float degreeOfPoint(std::vector<float>::iterator& ptr)
{
  long d = std::distance(g_laser.ranges.begin(), ptr);
  float bearing{ (g_laser.min + (d * g_laser.inc)) * G_CONVERSION_RATE };
  ROS_INFO("[robot_explorer] range=%.2f, position=%ld, angle_inc=%.2f", *ptr, d, bearing);
  return bearing;
}

float degreeOfFarPoint(std::vector<float>& ranges)
{
  auto far_pointer = (std::max_element(ranges.begin(), ranges.end()));
  return degreeOfPoint(far_pointer);
}

float degreeOfNearPoint(std::vector<float>& ranges)
{
  auto near_pointer = (std::min_element(ranges.begin(), ranges.end()));
  return degreeOfPoint(near_pointer);
}

int main(int argc, char** argv)
{
  // Initialize the ROS system and become a node.

  ros::init(argc, argv, "exploring");

  ROS_INFO("[robot_explorer] Robotic explorer node running and initialized! Let's have some fun!");

  ros::NodeHandle nh;
  // Create a subscriber object
  ros::Subscriber laser_sub = nh.subscribe("/laser_scan", 1000, processLaserCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, processOdomCallback);

  // Create a publisher object
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  // Seed the random number generator.
  srand(time(0));

  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);
  ros::Time begin = ros::Time::now();

  while (begin.toSec() == 0)
    begin = ros::Time::now();

  double ellapsed_time = 0;

  // loop variables
  geometry_msgs::Twist prev_msg;
  // loop init values
  rate.sleep();
  ros::spinOnce();  // give time to receive /base_scan messages
  while ((ros::ok()) && (ellapsed_time < 60 * 5))
  {
    // Create and fill in the message. The other four
    // fields , which are ignored by stage, default to 0.
    geometry_msgs::Twist msg;

    // # About default:
    // ## Turn to first far point ----------------
    msg.angular.z = 0.2 * (degreeOfFarPoint(g_laser.ranges) < 0 ? -1 : 1);

    // # About edges:
    // ## Edge finder ----------------
    // ### Right edge
    auto r_it_1{ g_laser.ranges.begin() };
    auto r_it_2{ g_laser.ranges.begin() };
    std::advance(r_it_2, 1);
    ROS_INFO("[robot_explorer] Looping for right edges...");
    while ((*r_it_2 - *r_it_1) < THRESHOLD and std::distance(r_it_2, g_laser.ranges.end()) > 0)
    {
      std::advance(r_it_2, 1);
      std::advance(r_it_1, 1);
    }
    bool right_edge{ (*r_it_2 - *r_it_1) >= THRESHOLD and *r_it_1 < THRESHOLD * 2 };
    ROS_INFO("[robot_explorer] Looped ---------");

    // ### Left edge
    auto l_it_1{ g_laser.ranges.end() };
    auto l_it_2{ g_laser.ranges.end() };
    std::advance(l_it_2, 1);
    ROS_INFO("[robot_explorer] Looping for left edges...");
    while ((*l_it_1 - *l_it_2) < THRESHOLD and std::distance(l_it_2, g_laser.ranges.end()) > 0)
    {
      std::advance(r_it_2, 1);
      std::advance(r_it_1, 1);
    }
    bool left_edge{ (*l_it_1 - *l_it_2) >= THRESHOLD and *r_it_1 < THRESHOLD * 2 };
    ROS_INFO("[robot_explorer] Looped ---------");

    // ## Edge behaviour ----------------
    if (right_edge or left_edge)
    {
      ROS_INFO("[robot_explorer] --------- Edge find ---------");
      float omega_sign;
      if (right_edge && (omega_sign = degreeOfPoint(r_it_2)) < 0)
      {
        omega_sign = -1;
      }
      else if (left_edge && (omega_sign = degreeOfPoint(l_it_2)) < 0)
      {
        omega_sign = -1;
      }
      else
      {
        omega_sign = 1;
      }
      msg.angular.z = 0.25 * omega_sign;
    }

    // # About doors:
    // ## Door finder ----------------
    auto it_12{ g_laser.ranges.begin() };
    std::advance(it_12, 1);
    auto it_11{ g_laser.ranges.begin() };
    while ((*it_12 - *it_11) < THRESHOLD and std::distance(it_12, g_laser.ranges.end()) > 0)
    {
      std::advance(it_12, 1);
      std::advance(it_11, 1);
    }
    bool first_edge{ (*it_12 - *it_11) >= THRESHOLD and *it_11 < THRESHOLD * 2 };

    auto it_22{ it_12 };
    std::advance(it_22, 1);
    auto it_21{ it_12 };
    while ((*it_21 - *it_22) < THRESHOLD and std::distance(it_22, g_laser.ranges.end()) > 0)
    {
      std::advance(it_22, 1);
      std::advance(it_21, 1);
    }
    bool second_edge{ (*it_21 - *it_22) >= THRESHOLD and *it_22 < THRESHOLD * 2 };

    // ## Door behaviour ----------------
    if (first_edge and second_edge)
    {
      ROS_INFO("[robot_explorer] --------- Door find ---------");
      std::advance(it_12, (std::distance(it_12, it_22)) / 2);
      float omega_sign = degreeOfPoint(it_12) < 0 ? -1 : 1;
      msg.angular.z = 0.4 * omega_sign;
      ROS_INFO("[robot_explorer] Going to: %.2f at %.2f", *it_12, degreeOfPoint(it_12));
    }

    // # About walls:
    // ## Reactive navigation for wall avoidance
    float close_point = (*(std::min_element(g_laser.ranges.begin(), g_laser.ranges.end())));
    if (close_point < 0.5)
    {
      ROS_INFO("[robot_explorer] --------- Avoiding crashing... ---------");
      // turn against near point
      float omega_sign = degreeOfNearPoint(g_laser.ranges) < 0 ? -1 : 1;
      msg.angular.z = 0.3 * (-omega_sign);
    }
    else
    {
      msg.linear.x = 0.8;
    }

    // # Common behaviour:
    // ## Publish the message.
    prev_msg = msg;
    pub.publish(msg);

    // ## Send a message to rosout with the details .
    ROS_INFO("[robot_explorer] Sending velocity command: linear= %.2f angular=%.2f", msg.linear.x, msg.angular.z);
    ROS_INFO("[robot_explorer] Front wall at = %.2f. Far point = %.2f", distanceToObsAt(FRONT_DEGREE),
             degreeOfFarPoint(g_laser.ranges));

    // ## Wait until it 's time for another iteration .
    rate.sleep();
    ros::spinOnce();  // give time to receive /base_scan messages

    ros::Time current = ros::Time::now();
    ellapsed_time = (current - begin).toSec();
    // ROS_INFO("[robot_explorer] Ellpased time: %.2f", ellapsed_time);
  }
}
