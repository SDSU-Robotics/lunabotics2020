#ifndef JOYMAP_H
#define JOYMAP_H

namespace JoyMap
{
    //TPort - axes
	static const int TPortForwardAxis = 1;
	static const int TPortTurnAxis = 4;

    //Excv - axes
    static const int ExcvLeftAxis = 1;
	static const int ExcvRightAxis = 4;	
    static const int ExcvTrencherUp = 5;
    static const int ExcvTrencherDown = 2;

    //TPort - buttons
    static const int TPortConveyorToggle = 0;
    static const int TPortToggleExtension = 7;

    //Excv - buttons
    static const int ExcvConveyorToggle = 0;
    static const int ExcvTrencherToggle = 1;
    static const int ExcvTrencherDriveToggle = 2;
    static const int ExcvTrencherDriveDecrease = 4;
    static const int ExcvTrencherDriveIncrease = 5;
    static const int ExcvTrencherExtend = 7;

    //Start semi Autonomy
    static const int SemiAutonomyToggle = 0;
};

#endif