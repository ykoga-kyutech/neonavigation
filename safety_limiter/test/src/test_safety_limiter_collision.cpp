/*
 * Copyright (c) 2018, the neonavigation authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtest/gtest.h>

namespace
{
void GenerateSinglePointPointcloud2(
    sensor_msgs::PointCloud2& cloud,
    const float x,
    const float y,
    const float z)
{
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  modifier.resize(1);
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
}
}  // namespace

class SafetyLimiterCollisionTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_vel_;
  ros::Publisher pub_cloud_;
  ros::Publisher pub_watchdog_;
  ros::Subscriber sub_cmd_vel_;
  tf2_ros::TransformBroadcaster tfb_;

  ros::Time cmd_vel_time_;

  void cbCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (cmd_vel_time_ == ros::Time(0))
      cmd_vel_time_ = now;
    const float dt = (now - cmd_vel_time_).toSec();

    yaw_ += msg->angular.z * dt;
    pos_ += Eigen::Vector2d(std::cos(yaw_), std::sin(yaw_)) * msg->linear.x * dt;
    cmd_vel_time_ = now;
  }

public:
  Eigen::Vector2d pos_;
  double yaw_;

  SafetyLimiterCollisionTest()
    : nh_("")
  {
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_in", 1);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    pub_watchdog_ = nh_.advertise<std_msgs::Empty>("watchdog_reset", 1);

    sub_cmd_vel_ = nh_.subscribe(
        "cmd_vel", 1, &SafetyLimiterCollisionTest::cbCmdVel, this);
  }
  void publishWatchdogReset()
  {
    std_msgs::Empty watchdog_reset;
    pub_watchdog_.publish(watchdog_reset);
  }
  void publishSinglePointPointcloud2(
      const float x,
      const float y,
      const float z,
      const std::string frame_id,
      const ros::Time stamp)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame_id;
    cloud.header.stamp = stamp;
    GenerateSinglePointPointcloud2(cloud, x, y, z);
    pub_cloud_.publish(cloud);
  }
  void publishTwist(
      const float lin,
      const float ang)
  {
    geometry_msgs::Twist cmd_vel_out;
    cmd_vel_out.linear.x = lin;
    cmd_vel_out.angular.z = ang;
    pub_cmd_vel_.publish(cmd_vel_out);
  }
  void publishTransform()
  {
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw_, Eigen::Vector3d(0, 0, 1)));
    geometry_msgs::TransformStamped trans;
    trans.header.frame_id = "odom";
    trans.header.stamp = ros::Time::now() + ros::Duration(0.1);
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = pos_[0];
    trans.transform.translation.y = pos_[1];
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    tfb_.sendTransform(trans);
  }
};

TEST_F(SafetyLimiterCollisionTest, StraitMotion)
{
  ros::Rate wait(20.0);

  // Skip initial state
  for (size_t i = 0; i < 10 && ros::ok(); ++i)
  {
    publishSinglePointPointcloud2(0.5, 0, 0, "odom", ros::Time::now());
    publishWatchdogReset();

    wait.sleep();
    ros::spinOnce();
  }

  for (float vel = 0.0; vel < 1.0; vel += 0.1)
  {
    bool en = false;

    // 1.0 m/ss, obstacle at 0.5 m: robot must not collide to the point
    for (size_t i = 0; i < 10 && ros::ok(); ++i)
    {
      publishTransform();

      if (i > 5)
        en = true;
      publishSinglePointPointcloud2(0.5, 0, 0, "odom", ros::Time::now());
      publishWatchdogReset();
      publishTwist(vel, 0.0);

      wait.sleep();
      ros::spinOnce();
    }
  }

  ASSERT_NEAR(yaw_, 0.0, 1e-2);
  ASSERT_LT(pos_[0], 0.5);
  ASSERT_NEAR(pos_[1], 0.0, 1e-2);
}

void timeSource()
{
  ros::NodeHandle nh("/");
  bool use_sim_time;
  nh.param("/use_sim_time", use_sim_time, false);
  if (!use_sim_time)
    return;

  ros::Publisher pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);

  ros::WallRate rate(500.0);  // 500% speed
  ros::WallTime time = ros::WallTime::now();
  while (ros::ok())
  {
    rosgraph_msgs::Clock clock;
    clock.clock.fromNSec(time.toNSec());
    pub.publish(clock);
    rate.sleep();
    time += ros::WallDuration(0.01);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_safety_limiter_collision");

  boost::thread time_thread(timeSource);

  return RUN_ALL_TESTS();
}
