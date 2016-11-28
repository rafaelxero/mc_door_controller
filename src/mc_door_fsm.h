#pragma once
#include "mc_door_controller.h"

namespace mc_door {

    struct DoorTaskState {
	
    public:

	DoorTaskState(const std::string & name);
	virtual ~DoorTaskState() {}

	DoorTaskState * update(DoorController & ctl);

	std::string name;

    protected:

	virtual void __init(DoorController & ctl) = 0;
	virtual DoorTaskState * __update(DoorController & ctl) = 0;

	bool first_call = true;
    };

    #define CREATE_STATE(NAME, DESC, MEMBERS) \
    struct NAME : public DoorTaskState {\
        NAME() : DoorTaskState(DESC) {}\
        virtual void __init(DoorController & ctl) override;\
        virtual DoorTaskState * __update(DoorController & ctl) override;\
        MEMBERS\
    };

    CREATE_STATE(IniState, "Initialization state",)
    CREATE_STATE(PrereachKnobState, "Pre-reach the knob with left hand state",)
    /*
    CREATE_STATE(OpenGripperState, "Open left gripper state",)
    CREATE_STATE(TouchDoorState, "Touch the door state",)
    CREATE_STATE(SeparateDoorState, "Separate from the door state",)
    CREATE_STATE(GraspKnobState, "Grasp the knob state",)
    CREATE_STATE(TurnKnobState, "Turn the knob state",)
    CREATE_STATE(PushDoorState, "Push the door state",)
    CREATE_STATE(ReturnKnobState, "Turn back the knob state",)
    CREATE_STATE(ReleaseKnobState, "Release the knob state",)
    CREATE_STATE(RetractArmState, "Retract the left arm state",)
    CREATE_STATE(GoHomeState, "Go back to home posture state",)
    */
    #undef CREATE_STEP
}
