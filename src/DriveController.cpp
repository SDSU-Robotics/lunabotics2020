#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/Joy.h>

using namespace std;

class Listener
{
public:
	void joyListener(const sensor_msgs::Joy::ConstPtr& Joy);
	void getJoyVals(bool buttons[], double axes[]) const;

private:
// initializes array values to off
    bool _buttons[12] = { 0 };
	double _axes[6] = { 0 };
};


void Listener::joyListener(const sensor_msgs::Joy::ConstPtr& Joy)
// gets on or off values for each button and axes from the joystick
{
	for (int i = 0 ; i < 12; i++)
		_buttons[i] = Joy->buttons[i];

    for (int i = 0; i < 6; i++)
        _axes[i] = Joy->axes[i];
}

void Listener::getJoyVals(bool buttons[], double axes[]) const
// sets each array value to on or off depending on button state
{
    for (int i = 0; i < 12; i++)
        buttons[i] = _buttons[i];

    for (int i = 0; i < 6; i++)
        axes[i] = _axes[i];
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DriveController");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber joySub = n.subscribe("joy", 100, &Listener::joyListener, &listener);
	
	bool buttons[12];
	double axes[6];

	ros::Publisher l_speed_pub = n.advertise<std_msgs::Float32>("ExcvLDrvPwr", 100); 
	// left speed of excavator drive power
    ros::Publisher r_speed_pub = n.advertise<std_msgs::Float32>("ExcvRDrvPwr", 100);
	// right speed of excavator drive power
	
    std_msgs::Float32 l_speed_msg; 
    std_msgs::Float32 r_speed_msg;
	
	while (ros::ok())
	{
        listener.getJoyVals(buttons, axes);

		// get controller values
		float speed = axes[1]; // left Y
		float turn = 1 * axes[3]; // right X

		// sets speed and turn values
		l_speed_msg.data = 0.75 * speed + 0.4 * turn;
		r_speed_msg.data = 0.75 * speed - 0.4 * turn;
		
		// publishes speed
		l_speed_pub.publish(l_speed_msg);
		r_speed_pub.publish(r_speed_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
