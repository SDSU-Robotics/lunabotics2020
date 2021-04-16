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
        void toCollector(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub);
        void toDig(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub);

        //Lists receiving msg data
        std::list<float> LSpeedList;
        std::list<float> RSpeedList;      

        //local variables initialize
        float lSpeed = 0;
        float rSpeed = 0;
        bool saveData = 0;


};


    
int main (int argc, char **argv)
{
    ros::init(argc, argv, "DugOdometry");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    DugOdometry dugOdometry;

    ros::Publisher lSpeedPub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
    ros::Publisher rSpeedPub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);   

    ros::Subscriber lSpeedSub = n.subscribe("TPortLDrvPwr", 100, &DugOdometry::getLSpeed, &dugOdometry);
	ros::Subscriber rSpeedSub = n.subscribe("TPortRDrvPwr", 100, &DugOdometry::getRSpeed, &dugOdometry);
    ros::Subscriber save_data_msg_sub = n.subscribe("SaveData", 100, &DugOdometry::setSaveData, &dugOdometry);

    while (ros::ok()) // runs while ros is running
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

float getListElement(std::list<float> l, int element)
{
    // get a pointer to the first element
    auto position = l.begin();
    // move the pointer to the desired element
    advance(position, element);
    
    // Returns task at pointer 
    return *position;   
}

//set list values to msg data
void DugOdometry::getLSpeed(const std_msgs::Float32 lspeed)
{

    lSpeed = lspeed.data;

}

//set list values to msg data
void DugOdometry::getRSpeed(const std_msgs::Float32 rspeed)
{

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

void toDig(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub)
{
    std_msgs::Float32 lspeedmsg;
    std_msgs::Float32 rspeedmsg;

    int size;

    if(LSpeedList.size() < RSpeedList.size())
        size = LSpeedList.size();
    else
        size = RSpeedList.size();

    for(int l = 0; l <= size; l++)
    {
        lspeedmsg.data = getListElement(LSpeedList, l);
        lSpeedPub.publish(lspeedmsg);
    }

        for(int r = 0; r <= size; r++)
    {
        rspeedmsg.data = getListElement(RSpeedList, r);
        rSpeedPub.publish(rspeedmsg);
    }

}

void toCollector(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub)
{
    std_msgs::Float32 lspeedmsg;
    std_msgs::Float32 rspeedmsg;

    int size;

    if(LSpeedList.size() < RSpeedList.size())
        size = LSpeedList.size();
    else
        size = RSpeedList.size();


    for(int l = size; l >= 1; l--)
    {
        lspeedmsg.data = getListElement(LSpeedList, l) * -1;
        lSpeedPub.publish(lspeedmsg);
    }


    for(int r = size; r >= 1; r--)
    {
        rspeedmsg.data = getListElement(RSpeedList, r) * -1;
        rSpeedPub.publish(rspeedmsg);
    }

}