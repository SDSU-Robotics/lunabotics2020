#ifndef JOYMAP_H
#define JOYMAP_H

namespace JoyMap
{
    //TPort - axes
	static const int TPortForwardAxis = 1;
	static const int TPortTurnAxis = 4;

    //Excv - axes
    static const int ExcvLeftAxis = 1;  //Dig Left Motor
	static const int ExcvRightAxis = 4;	//Dig Right Motor
    static const int ExcvTrencherUp = 5;    //Dig Trencher Pivot Up
    static const int ExcvTrencherDown = 2;  //Dig Trencher Pivot Down

    //TPort - buttons
    static const int TPortConveyorToggle = 0;
    static const int TPortToggleExtension = 7;

    //Excv - buttons
    static const int ExcvConveyorToggle = 0;  //Toggle Dig Conveyor On/Off - 1/0
    static const int ExcvTrencherToggle = 1;  //Toggle Dig Autonomous Trenching On/Off - 1/0
    static const int ExcvTrencherDriveToggle = 2;       
    static const int ExcvTrencherDriveDecrease = 4;     //Dig trencher speed decrease
    static const int ExcvTrencherDriveIncrease = 5;     //Dig trencher speed increase
    static const int ExcvTrencherExtend = 7;            //Toggle trencher linear actuator In/Out - 0/1
};

#endif