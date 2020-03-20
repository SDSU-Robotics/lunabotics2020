#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

using namespace std;

class DugHW : public hardware_interface::RobotHW
{
    public:
        DugHW()
        {
            hardware_interface::JointStateHandle drive_left_front_handle("drive_left_front", &pos[0], &vel[0], &eff[0]);
            drive_left_front_state_interface.registerHandle(drive_left_front_handle);

            registerInterface(&drive_left_front_state_interface);

            hardware_interface::JointHandle drive_left_front_vel_handle(drive_left_front_vel_interface.getHandle("drive_left_front"), &cmd[0]);
            drive_left_front_vel_interface.registerHandle(drive_left_front_vel_handle);

            registerInterface(&drive_left_front_vel_interface);
        } 
    private:
        hardware_interface::JointStateInterface drive_left_front_state_interface;
        hardware_interface::VelocityJointInterface drive_left_front_vel_interface;
        double cmd[1];
        double pos[1];
        double vel[1];
        double eff[1];

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "DugHW");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	while (ros::ok())
	{	
		ros::spinOnce();
		loop_rate.sleep();
	}
}

