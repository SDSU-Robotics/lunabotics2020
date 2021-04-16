#include <list>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

class DugOdometry
{
    public: 


        //obtaining msg data
        void getLSpeed(const std_msgs::Float32 lspeed);
        void getRSpeed(const std_msgs::Float32 rspeed);
        void setSaveData(const std_msgs::Bool saving);
        void save();
        void saveReversed(/*std::list<float> LSpeedList, std::list<float> RSpeedList*/);        //takes lists from save and reverses them
        

        //Lists receiving msg data
        std::list<float> LSpeedList;
        std::list<float> RSpeedList;

        //Lists being manipulated
        std::list<float> RevLSpeedList;
        std::list<float> RevRSpeedList;        

        //local variables initialize
        float lSpeed = 0;
        float rSpeed = 0;
        bool saveData = false;


};


    
int main (int argc, char **argv)
{
    ros::init(argc, argv, "DugOdometry");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    DugOdometry dugOdometry;

    ros::Subscriber lSpeedSub = n.subscribe("TPortLDrvPwr", 100, &DugOdometry::getLSpeed, &dugOdometry);
	ros::Subscriber rSpeedSub = n.subscribe("TPortRDrvPwr", 100, &DugOdometry::getRSpeed, &dugOdometry);
    ros::Subscriber save_data_msg_sub = n.subscribe("SaveData", 100, &DugOdometry::setSaveData, &dugOdometry);

    while (ros::ok()) // runs while ros is running
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//set list values to msg data
void DugOdometry::getLSpeed(const std_msgs::Float32 lspeed)
{

	//LSpeedList.push_back(lspeed);
    lSpeed = lspeed.data;

}

//set list values to msg data
void DugOdometry::getRSpeed(const std_msgs::Float32 rspeed)
{

	//LSpeedList.push_back(lspeed);
    rSpeed = rspeed.data;

}


//set local variable to msg data
void DugOdometry::setSaveData(const std_msgs::Bool saving)
{
    saveData = saving.data;

}


void DugOdometry::save()
{
    if(saveData == 1)
    {
        LSpeedList.push_back(lSpeed);
        RSpeedList.push_back(rSpeed);
    }
   
}

void DugOdometry::saveReversed(/*std::list<float> LSpeedList, std::list<float> RSpeedList*/)
{
    if (saveData == 1)
    {
        RevLSpeedList.push_front(lSpeed * -1);
        RSpeedList.push_front(rSpeed * -1);        
    }

    /*int i;

    for(i = LSpeedList.size(); i >= 1; i--)
    {
        RevLSpeedList = -1 * LSpeedList(i);
    }

    */
}
