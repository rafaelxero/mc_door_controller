#include <mc_rtc/logging.h>
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

	ctl.postureTask->stiffness(1.0);

	std::vector<std::string> unactiveJoints = {"CHEST_JOINT1"};

	ctl.orBodyTask = std::make_shared<mc_tasks::OrientationTask>("BODY", ctl.robots(), 0, 2.0, 50.0);

	ctl.efTask = std::make_shared<mc_tasks::EndEffectorTask>("LARM_LINK6", ctl.robots(), 0, 5.0, 100.0);

        ctl.efTask->selectUnactiveJoints(ctl.solver(), unactiveJoints);

	auto & door = ctl.robots().robots()[1];

	ctl.orBodyTask->orientation(Eigen::Matrix3d::Identity());

	sva::PTransformd offset(sva::RotX(-M_PI/2), Eigen::Vector3d(0, 0.4, 0));
	ctl.efTask->set_ef_pose(offset*door.surface("Handle").X_0_s(door));
	
	ctl.solver().addTask(ctl.comTask);
	ctl.solver().addTask(ctl.orBodyTask);
	ctl.solver().addTask(ctl.efTask);

	ctl.solver().setContacts({
		{ctl.robots(), 0, 2, "LFullSole", "AllGround"},
		{ctl.robots(), 0, 2, "RFullSole", "AllGround"}
	    });
    }

    DoorTaskState * PrereachKnobState::__update(DoorController & ctl) {
	
	if (ctl.efTask->eval().norm() < 1e-1) {
	    return new OpenGripperState;
	}

	return this;
    }

    void OpenGripperState::__init(DoorController & /*ctl*/) {
	
	targetOpen = 1.0;
	openSpeed = 0.002;
    }

    DoorTaskState * OpenGripperState::__update(DoorController & ctl) {

	auto lG = ctl.grippers.at("l_gripper");
	
	if (lG->percentOpen[0] >= targetOpen) {
	    lG->percentOpen[0] = targetOpen;
	    return new ReachDoorState;
	}
	else {
	    lG->percentOpen[0] += openSpeed;
	    return this;
	}
    }

    void ReachDoorState::__init(DoorController & ctl) {

	auto & door = ctl.robots().robots()[1];
	
	sva::PTransformd offset(sva::RotX(-M_PI/2), Eigen::Vector3d(0, 0.22, 0));
	ctl.efTask->set_ef_pose(offset*door.surface("Handle").X_0_s(door));

	ctl.solver().addTask(ctl.efTask);
    }

    DoorTaskState * ReachDoorState::__update(DoorController & ctl) {

	if (ctl.efTask->eval().norm() < 5e-2 && ctl.efTask->speed().norm() < 1e-1) {
	    return new TouchDoorState;
	}

	return this;
    }
    
    void TouchDoorState::__init(DoorController & ctl) {

	ctl.solver().removeTask(ctl.efTask);
	ctl.solver().removeConstraintSet(ctl.doorCollisionsConstraint);

	const mc_rbdyn::ForceSensor& forceSensor = ctl.robots().robot().forceSensorData("LeftHandForceSensor");

	ctl.compTask = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(ctl.robots(), 0, forceSensor, ctl.getWrenches(), ctl.calibrator, ctl.timeStep, 3.0, 1000));
	
	ctl.compTask->setTargetWrench(sva::ForceVecd(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0.0, 0, 30.0)));

	ctl.solver().addTask(ctl.compTask);

	LOG_INFO("Here");
    }

    DoorTaskState * TouchDoorState::__update(DoorController & ctl) {
	
	if (ctl.compTask->eval().norm() < 25 && ctl.compTask->speed().norm() < 1e-1) {

	    ctl.solver().removeTask(ctl.compTask);
	    ctl.solver().addTask(ctl.efTask);
	    ctl.efTask->reset();

	    ctl.solver().setContacts({
		    {ctl.robots(), 0, 2, "LFullSole", "AllGround"},
		    {ctl.robots(), 0, 2, "RFullSole", "AllGround"},
		    {ctl.robots(), 0, 1, "LeftFingers", "Door"}
		});

	    return new SeparateDoorState;
	}
	
	return this;
    }

    void SeparateDoorState::__init(DoorController & ctl) {
	
	Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
	dof(5,5) = 0;

	contactId = mc_rbdyn::Contact(ctl.robots(), 0, 1, "LeftFingers", "Door").contactId(ctl.robots());

	ctl.contactConstraint.contactConstr->addDofContact(contactId, dof);
	ctl.contactConstraint.contactConstr->updateDofContacts();

	ctl.efTask->add_ef_pose(sva::PTransformd(Eigen::Vector3d(-0.05, 0, 0)));
    }

    DoorTaskState * SeparateDoorState::__update(DoorController & ctl) {

	if (ctl.efTask->eval().norm() < 1e-2) {

	    ctl.contactConstraint.contactConstr->removeDofContact(contactId);
	    ctl.contactConstraint.contactConstr->updateDofContacts();

	    ctl.solver().setContacts({
		    {ctl.robots(), 0, 2, "LFullSole", "AllGround"},
		    {ctl.robots(), 0, 2, "RFullSole", "AllGround"}
		});

	    return new GraspKnobState;
	}

	return this;
    }

    void GraspKnobState::__init(DoorController & /*ctl*/) {

	targetClose = 0.0;
	closeSpeed = 0.002;
    }

    DoorTaskState * GraspKnobState::__update(DoorController & ctl) {

	auto lG = ctl.grippers.at("l_gripper");
	
	if (lG->overCommandLimit[0] || lG->percentOpen[0] <= targetClose) {

	    ctl.solver().setContacts({
		    {ctl.robots(), 0, 2, "LFullSole", "AllGround"},
		    {ctl.robots(), 0, 2, "RFullSole", "AllGround"},
		    {ctl.robots(), 0, 1, "LeftFingers", "Handle"}
		});

	    ctl.solver().removeTask(ctl.efTask);

	    return new TurnKnobState;
	}
	else {
	    lG->percentOpen[0] -= closeSpeed;
	    return this;
	}

	return this;
    }

    void TurnKnobState::__init(DoorController & ctl) {
	
	auto & door = ctl.robots().robots()[1];

	ctl.doorPostureTask = std::make_shared<tasks::qp::PostureTask>(ctl.robots().mbs(), 1, door.mbc().q, 1.0, 100.0);

	ctl.solver().addTask(ctl.doorPostureTask);

	targetAngle = 0.5236;
	turnSpeed = 0.01;
    }

    DoorTaskState * TurnKnobState::__update(DoorController & ctl) {

	auto qDoor = ctl.robots().robots()[1].mbc().q;
	auto qTarget = ctl.doorPostureTask->posture();

	if (qDoor[2][0] > targetAngle) {

	    ctl.solver().removeTask(ctl.doorPostureTask);
	    ctl.solver().addTask(ctl.efTask);
	    ctl.efTask->reset();

	    return new PushDoorState;
	}
	else {
	    qTarget[2][0] += turnSpeed;
	    ctl.doorPostureTask->posture(qTarget);
	}

	return this;
    }

    void PushDoorState::__init(DoorController & ctl) {

	std::vector<tasks::qp::JointGains> gains = {{"CHEST_JOINT1", 10.0}};

	ctl.postureTask->posture(ctl.robots().robot().mbc().q);
	ctl.postureTask->jointsGains(ctl.robots().mbs(), gains);
	ctl.postureTask->weight(10.0);

	ctl.solver().addTask(ctl.doorPostureTask);

	targetAngle = -0.05;
	pushSpeed = 0.01;

	//auto qTarget = ctl.doorPostureTask->posture();
	//qTarget[1][0] = targetAngle;
	//ctl.doorPostureTask->posture(qTarget);
    }

    DoorTaskState * PushDoorState::__update(DoorController & ctl) {

	auto qDoor = ctl.robots().robots()[1].mbc().q;
	auto qTarget = ctl.doorPostureTask->posture();

	if (qDoor[1][0] < targetAngle) {

	    ctl.solver().removeTask(ctl.doorPostureTask);
	    ctl.efTask->reset();

	    return new ReturnKnobState;
	}
	else {
	    //ctl.doorPostureTask->weight(ctl.doorPostureTask->weight() + 0.01);
	    qTarget[1][0] -= pushSpeed;
	    ctl.doorPostureTask->posture(qTarget);
	}

	return this;
    }

    void ReturnKnobState::__init(DoorController & ctl) {

	ctl.solver().addTask(ctl.doorPostureTask);

	targetAngle = 0.0;
	turnSpeed = 0.01;
    }

    DoorTaskState * ReturnKnobState::__update(DoorController & ctl) {

	auto qDoor = ctl.robots().robots()[1].mbc().q;
	auto qTarget = ctl.doorPostureTask->posture();

	if (qDoor[2][0] < targetAngle) {

	    ctl.solver().removeTask(ctl.doorPostureTask);
	    ctl.solver().addTask(ctl.efTask);
	    ctl.efTask->reset();

	    return new ReleaseKnobState;
	}
	else {
	    qTarget[2][0] -= turnSpeed;
	    ctl.doorPostureTask->posture(qTarget);
	}

	return this;
    }

    void ReleaseKnobState::__init(DoorController & /*ctl*/) {

	targetOpen = 1.0;
	openSpeed = 0.002;
    }

    DoorTaskState * ReleaseKnobState::__update(DoorController & ctl) {

	auto lG = ctl.grippers.at("l_gripper");
	
	if (lG->percentOpen[0] >= targetOpen) {

	    lG->percentOpen[0] = targetOpen;

	    ctl.solver().setContacts({
		    {ctl.robots(), 0, 2, "LFullSole", "AllGround"},
		    {ctl.robots(), 0, 2, "RFullSole", "AllGround"}
		});

	    return nullptr;
	}
	else {
	    lG->percentOpen[0] += openSpeed;
	    return this;
	}
    }

    /*
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
