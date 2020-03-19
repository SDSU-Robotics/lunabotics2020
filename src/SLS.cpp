#include <iostream>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <std_msgs/Float32.h>
#include <math.h>

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#define PI 3.141592653589

#define SLS_HEIGHT 0.098 // meters
#define LEVER_LENGTH 0.005 // meters

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

#define MIN_PULSE 500.0	// us
#define MAX_PULSE 2250.0	// us
#define MIN_INPUT 0
#define MAX_INPUT PI / 2

class Listener
{
public:
	void setPosition(const std_msgs::Float32);
	void setChildFrame(string frame) { _childFrame = frame; }
	void setParentFrame(string frame) { _parentFrame = frame; }

private:
	CANifier _canifer = {1};
	string _childFrame;
	string _parentFrame;
};

int main(int argc, char** argv){
	ros::init(argc, argv, "SLS");
	
	ros::NodeHandle n;

	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	string frame;
	n.param<std::string>("child_frame", frame, "/SLS_lidar_frame");
	listener.setChildFrame(frame);

	n.param<std::string>("parent_frame", frame, "/map");
	listener.setParentFrame(frame);

	ros::Subscriber pos_sub = n.subscribe("sls_pos", 1000, &Listener::setPosition, &listener);

	ros::spin();

	return 0;
};

void Listener::setPosition(const std_msgs::Float32 msg)
{
	// limit values
	float pos = msg.data;
	if (pos < MIN_INPUT)	pos = MIN_INPUT;
	if (pos > MAX_INPUT)	pos = MAX_INPUT;

	_canifer.SetGeneralOutput(CANifier::GeneralPin::SPI_CLK_PWM0P, false, true);

	float pulse = LinearInterpolation::Calculate(pos, MIN_INPUT, MIN_PULSE, MAX_INPUT, MAX_PULSE); // pulse length in us

	_canifer.SetPWMOutput(0, pulse / 4200.0); // 4.2 ms period
	
	_canifer.EnablePWMOutput(0, true);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;

	float pitch =  pos; // radians

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = _parentFrame;
	transformStamped.child_frame_id = _childFrame;
	transformStamped.transform.translation.x = 0.0;
	transformStamped.transform.translation.y = LEVER_LENGTH * cos(pitch);
	transformStamped.transform.translation.z = SLS_HEIGHT + LEVER_LENGTH * sin(pitch);

	tf2::Quaternion q;
	q.setRPY(0.0, -1 * pitch, 0.0);

	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();

	br.sendTransform(transformStamped);
}
