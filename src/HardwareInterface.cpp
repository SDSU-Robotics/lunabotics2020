#include "ctre/Phoenix.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

class Listener
{
    public:
        void lSpeedListener(const std_msgs::Float32 lspeed) const;
        void rSpeedListener(const std_msgs::Float32 rspeed) const;

    private:
        talonSRX lTalon = 2;
        talonSRX rTalon = 1;
}

void lSpeedListener(const std_msgs::Float32 lspeed) const
{
    lTalon.set(ControlMode::PercentOutput, lspeed);
}

void rSpeedListener(const std_msgs::Float32 rspeed) const
{
    rTalon.set(ControlMode::PercentOutput, rspeed);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "HardwareInterface");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	Listener listener;

	ros::Subscriber speedSub = n.subscribe("l_speed", 100, &Listener::joyListener, &listener);
	ros::Subscriber speedSub = n.subscribe("r_speed", 100, &Listener::joyListener, &listener);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}