#include "mc_door_fsm.h"

namespace mc_door {

    DoorTaskState::DoorTaskState(const std::string & name)
	: name(name)
    {}

    DoorTaskState * DoorTaskState::update(DoorController & ctl) {
	
	if (first_call) {
	    __init(ctl);
	    first_call = false;
	    return this;
	}
	return __update(ctl);
    }

    void IniState::__init(DoorController & ctl) {
	
	ctl.comTask = std::make_shared<mc_tasks::CoMTask>(ctl.robots(), ctl.robots().robotIndex(), 3.0, 100.0);
	auto mbc = ctl.robot().mbc();
	mbc.q = ctl.postureTask->posture();
	auto comT = rbd::computeCoM(ctl.robot().mb(), mbc);
	ctl.comTask->com(comT);
	ctl.solver().addTask(ctl.comTask);
    }

    DoorTaskState * IniState::__update(DoorController & ctl) {
	
	if (ctl.comTask->speed().norm() < 1e-2) {
	    return new PrereachKnobState;
	}

	return this;
    }

    void PrereachKnobState::__init(DoorController & ctl) {
    }

    DoorTaskState * PrereachKnobState::__update(DoorController & ctl) {

	return this;
    }

    /*
    void OpenGripperState::__init(DoorController & ctl) {
    }

    DoorTaskState * OpenGripperState::__update(DoorController & ctl) {

	return this;
    }

    void TouchDoorState::__init(DoorController & ctl) {
    }

    DoorTaskState * TouchDoorState::__update(DoorController & ctl) {

	return this;
    }

    void SeparateDoorState::__init(DoorController & ctl) {
    }

    DoorTaskState * SeparateDoorState::__update(DoorController & ctl) {

	return this;
    }

    void GraspKnobState::__init(DoorController & ctl) {
    }

    DoorTaskState * GraspKnobState::__update(DoorController & ctl) {

	return this;
    }

    void TurnKnobState::__init(DoorController & ctl) {
    }

    DoorTaskState * TurnKnobState::__update(DoorController & ctl) {

	return this;
    }

    void PushDoorState::__init(DoorController & ctl) {
    }

    DoorTaskState * PushDoorState::__update(DoorController & ctl) {

	return this;
    }

    void ReturnKnobState::__init(DoorController & ctl) {
    }

    DoorTaskState * ReturnKnobState::__update(DoorController & ctl) {

	return this;
    }

    void ReleaseKnobState::__init(DoorController & ctl) {
    }

    DoorTaskState * ReleaseKnobState::__update(DoorController & ctl) {

	return this;
    }

    void RetractArmState::__init(DoorController & ctl) {
    }

    DoorTaskState * RetractArmState::__update(DoorController & ctl) {

	return this;
    }

    void GoHomeState::__init(DoorController & ctl) {
    }

    DoorTaskState * GoHomeState::__update(DoorController & ctl) {

	return this;
    }
    */
}
