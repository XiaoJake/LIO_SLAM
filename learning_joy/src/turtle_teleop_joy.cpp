#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose.h>
#include <stdlib.h>
#include <stdio.h> 
#include <thread>  
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <iostream>
#include <ros/console.h>

using namespace std;  

class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	//void odomCallback(const turtlesim::Pose::ConstPtr& turtlePose);
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
	bool flag=false,button_up=false,button_up1=false;
	int count=1;

  ros::Publisher vel_pub_;
	ros::Publisher pose_pub_;
  ros::Subscriber joy_sub_;
	ros::Subscriber odom_sub_;
};

TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	pose_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_data", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopTurtle::joyCallback, this);
  //odom_sub_ = nh_.subscribe<turtlesim::Pose>("turtle1/pose", 1, &TeleopTurtle::odomCallback, this);
	odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("transformed_odom", 1, &TeleopTurtle::odomCallback, this);
}

void rostopic()
{
	//system("rostopic echo /pose_data > /home/a/catkin_ws/pose_data.txt");
	system("rosbag record -O /home/xiaojake/corner.bag /odom_data __name:=my_record_node1");
}

void rosbag_command()
{
	system("rosbag record -O /home/xiaojake/lidar_imu.bag /imu/data /velodyne_points __name:=my_record_node2");
	//system("rosbag record -O /home/a/lidar_imu.bag /joy /turtle1/pose __name:=my_record_node2");
}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if(joy->axes[2]==-1&&joy->axes[5]==-1||button_up)  //button LT+RT
	{
		button_up=true;
		if(joy->axes[5]==1)
		{
			flag=true;
			button_up=false;
		}
	}
	if(joy->axes[2]==-1&&joy->buttons[7]&&button_up1==false)  //button LT+START(shart rosbag)
	{
		/*//std::string path = "/home/a/test.bag";
		std::string path = "/home/robot/test.bag";
		//std::string topics = " /turtle1/pose /joy";
		std::string topics = " /imu/data /velodyne_points";
		std::string node_name = " __name:=my_record_node2";
		std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
		int ret = system(cmd_str.c_str()); */
		button_up1=true;
		thread task2(rosbag_command);  
		task2.detach();
	}
	if(joy->axes[5]==-1&&joy->buttons[7])  //button LB+START(end rosbag)
	{
		/*ros::V_string v_nodes;
		ros::master::getNodes(v_nodes);
		std::string node_name = std::string("/my_record_node2");
		auto it = std::find(v_nodes.begin(), v_nodes.end(), node_name.c_str());
		if (it != v_nodes.end())
		{
			std::string cmd_str = "rosnode kill " + node_name;
			int ret = system(cmd_str.c_str());
			std::cout << "## stop rosbag record cmd: " << cmd_str << std::endl;
		}*/
		system("rosnode kill my_record_node2");
	}

  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}

/*
void TeleopTurtle::odomCallback(const turtlesim::Pose::ConstPtr& turtlePose)
{
	if(flag==true)
	{
		flag=false;
		turtlesim::Pose pose;
		pose=*turtlePose;
		pose_pub_.publish(pose);
		count++;
		cout << "corner_msg received" << endl;
		cout << "corner_msg_" << count-1 << endl;
		if(count>12)
			system("rosnode kill my_record_node1");
	}
}
*/

void TeleopTurtle::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	if(flag==true)
	{
		flag=false;
		nav_msgs::Odometry pose;
		pose=*odom;
		pose_pub_.publish(pose);
		count++;
		cout << "corner_msg received" << endl;
		cout << "corner_msg_" << count-1 << endl;
		if(count>12)
			system("rosnode kill my_record_node1");
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;
	
	thread task1(rostopic);  
	task1.detach();

	//std::string path = "/home/a/corner.bag";
	/*std::string path = "/home/robot/corner.bag";
	std::string topics = " /pose_data";
	std::string node_name = " __name:=my_record_node1";
	std::string cmd_str = "gnome-terminal -x bash -c 'rosbag record -O " + path + topics + node_name + "'";
	int ret = system(cmd_str.c_str()); */

	ros::spin();
}






