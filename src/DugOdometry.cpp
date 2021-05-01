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
        void setDigData(const std_msgs::Bool saving);
        void setCollectData(const std_msgs::Bool saving);
        void dig(DugOdometry dugOdometry);
        void collect(DugOdometry dugOdometry);
        int getListSize(int size);
        float getTimeTraveled(float timeTravel, int size);


        void save();
        void toCollector(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub, ros::Publisher CollectPub);
        void toDig(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub, ros::Publisher DigPub);

        //Lists receiving msg data
        std::list<float> LSpeedList;
        std::list<float> RSpeedList;      

        //variables initialize
        float lSpeed = 0;
        float rSpeed = 0;

        int size = 0;

        bool saveData = 0;
        bool saveDigData = 0;
        bool saveCollectData = 0;
        std_msgs::Bool saveDigDataMsg;
        std_msgs::Bool saveCollectDataMsg;
        std_msgs::Bool enablePubMsg;
        std_msgs::Bool disableCallToMsg;
};


    
int main (int argc, char **argv)
{
    ros::init(argc, argv, "DugOdometry");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

    bool saveDataLast;

    DugOdometry dugOdometry;

    ros::Publisher lSpeedPub = n.advertise<std_msgs::Float32>("TPortLDrvPwr", 100);
    ros::Publisher rSpeedPub = n.advertise<std_msgs::Float32>("TPortRDrvPwr", 100);   
    ros::Publisher enablePubPub = n.advertise<std_msgs::Bool>("EnableSpeedPub", 100);
    ros::Publisher disableCallToPub = n.advertise<std_msgs::Bool>("DisableCallTo", 100);    

    ros::Subscriber lSpeedSub = n.subscribe("TPortLDrvPwr", 100, &DugOdometry::getLSpeed, &dugOdometry);
	ros::Subscriber rSpeedSub = n.subscribe("TPortRDrvPwr", 100, &DugOdometry::getRSpeed, &dugOdometry);
    ros::Subscriber save_data_msg_sub = n.subscribe("SaveOdomData", 100, &DugOdometry::setSaveData, &dugOdometry);
    ros::Subscriber digData_msg_sub = n.subscribe("DigData", 100, &DugOdometry::setDigData, &dugOdometry);
    ros::Subscriber collectorData_msg_sub = n.subscribe("CollectData", 100, &DugOdometry::setCollectData, &dugOdometry);



    while (ros::ok()) // runs while ros is running
	{
        /*if(!saveDataLast&&dugOdometry.saveData)
        {
            ROS_INFO("TEST");
            std::cout << "test"<< std::endl;
            dugOdometry.LSpeedList.clear();
            dugOdometry.RSpeedList.clear();
        }*/
        //if(dugOdometry.saveData)
          //  dugOdometry.save();
        if(dugOdometry.saveDigData)
        {
            dugOdometry.toDig(dugOdometry.LSpeedList, dugOdometry.RSpeedList, lSpeedPub, rSpeedPub, enablePubPub);
           
            dugOdometry.disableCallToMsg.data = true;
            disableCallToPub.publish(dugOdometry.disableCallToMsg);     //tells tport to update digdata to 0 so function stops running

            ros::Duration(0.1).sleep();     //allows message to update in TPort Controller

            dugOdometry.disableCallToMsg.data = false;
            disableCallToPub.publish(dugOdometry.disableCallToMsg);

            dugOdometry.enablePubMsg.data = true;
            enablePubPub.publish(dugOdometry.enablePubMsg);     //tells tport to publish motor speed again
        }

        if(dugOdometry.saveCollectData)
        {
            dugOdometry.toCollector(dugOdometry.LSpeedList, dugOdometry.RSpeedList, lSpeedPub, rSpeedPub, enablePubPub);

            dugOdometry.disableCallToMsg.data = true;
            disableCallToPub.publish(dugOdometry.disableCallToMsg);     //tells tport to update digdata to 0 so function stops running

            dugOdometry.enablePubMsg.data = true;
            enablePubPub.publish(dugOdometry.enablePubMsg);     //tells tport to publish motor speed again
        }

		ros::spinOnce();
		loop_rate.sleep();
        saveDataLast = dugOdometry.saveData;
        std::cout << saveDataLast << std::endl;
        std::cout << " " << dugOdometry.saveData << std::endl;
	}
}

int getListSize(int size, std::list<float> LSpeedList, std::list<float> RSpeedList)
{
    if(LSpeedList.size() < RSpeedList.size())
        size = LSpeedList.size();
    else
        size = RSpeedList.size();

        return size;
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
    if(saveData)
    {
        lSpeed = lspeed.data;
        LSpeedList.push_back(lSpeed);
    }

}

//set list values to msg data
void DugOdometry::getRSpeed(const std_msgs::Float32 rspeed)
{
    if(saveData)
    {
        rSpeed = rspeed.data;
        RSpeedList.push_back(rSpeed);
    }
}


//set local variable to msg data
void DugOdometry::setSaveData(const std_msgs::Bool saving)
{
    saveData = saving.data;

}
void DugOdometry::setDigData(const std_msgs::Bool saving)
{
    saveDigData = saving.data;

}
void DugOdometry::setCollectData(const std_msgs::Bool saving)
{
    saveCollectData = saving.data;

}

void DugOdometry::save()
{
    if(saveData == 1)
    {
        LSpeedList.push_back(lSpeed);
        RSpeedList.push_back(rSpeed);
        ros::Duration(1/100).sleep();
    }
   
}


void DugOdometry::toDig(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub, ros::Publisher enablePubPub)
{
    std_msgs::Float32 lspeedmsg;
    std_msgs::Float32 rspeedmsg;

    int size;

    if(LSpeedList.size() < RSpeedList.size())
    {
        size = LSpeedList.size();
    }

    else
    {
        size = RSpeedList.size();
    }

    for(int i = 0; i <= size; i++)
    {
        lspeedmsg.data = getListElement(LSpeedList, i);
        rspeedmsg.data = getListElement(RSpeedList, i);
        lSpeedPub.publish(lspeedmsg);
        rSpeedPub.publish(rspeedmsg);    
        usleep(10000);
        ros::spinOnce();        //Publishes during the same ROS Spin cycle as needed by TPortController::callTo

    }


}

void DugOdometry::toCollector(std::list<float> LSpeedList, std::list<float> RSpeedList, ros::Publisher lSpeedPub, ros::Publisher rSpeedPub, ros::Publisher enablePubPub)
{
    std_msgs::Float32 lspeedmsg;
    std_msgs::Float32 rspeedmsg;

    int size;

    if(LSpeedList.size() < RSpeedList.size())
    {
        size = LSpeedList.size();
    }

    else
    {
        size = RSpeedList.size();
    }

    
  //Publishes during the same ROS Spin cycle as needed by TPortController::callTo


    for(int i = size; i >= 1; i--)
    {
        lspeedmsg.data = getListElement(LSpeedList, i) * -1;
        rspeedmsg.data = getListElement(RSpeedList, i) * -1;
        lSpeedPub.publish(lspeedmsg);
        rSpeedPub.publish(rspeedmsg);
        usleep(10000);      
        ros::spinOnce();        //Publishes during the same ROS Spin cycle as needed by TPortController::callTo

    }

}