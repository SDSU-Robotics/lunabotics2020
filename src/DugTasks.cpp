#include "ros/ros.h"
#include "std_msgs/Float32.h" 
#include "std_msgs/UInt16.h"
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"

class DugTasks
{
    private:
        // listeners are private
        bool prime(); //prepare flags and lin act
        bool drvTrench; //drive to beginning of current trench being excavated
        bool collectGravel; //drive straight until docked with Dig and flipped gravel switch
        bool drvCollector; //reverse over trench and traverse to collector
        bool alignCollector; //align with collector and dock collector
        bool dumpGravel; //activate belt to transfer gravel to collector
        std_msgs::UInt16 LinearActuatorExtendPwrMsg;

    public:
        //void setLinearActuatorExtendSpeed(const std_msgs::Int8 msg);
        //void setLinearActuatorExtendPos(std_msgs::UInt16 &extend_pos);
        //float extendSpeed = 0;
        //float linearActuatorExtendPos = 0;


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DugTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    DugTasks dugTasks;
    ros::Publisher ExtendLinearActuatorExtendPwrPub = n.advertise<std_msgs::UInt16>("TPortExtendPwr", 100);
    //ros::Subscriber ExtendLinearActuatorExtendPosSub = n.subscribe("TPortExtendPwr", 100, &DugTasks::setLinearActuatorExtendPos, &dugTasks);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}


//prepare flags and lin act
bool DugTasks::prime()
{
    //set msg to 1
    LinearActuatorExtendPwrMsg.data = 1;

}

/*
//drive to beginning of current trench being excavated
bool DugTasks::drvTrench()
{

}

//drive straight until docked with Dig and flipped gravel switch
bool DugTasks::collectGravel()
{

}

//reverse over trench and traverse to collector
bool DugTasks::drvCollector()
{

}

//align with collector and dock collector
bool DugTasks::alignCollector()
{

}

//activate belt to transfer gravel to collector
bool DugTasks::dumpGravel()
{

}
*/