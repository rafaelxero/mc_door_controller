#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_rbdyn/calibrator.h>
#include <RBDyn/FK.h>

namespace mc_door {

    struct DoorTaskState;

    struct DoorController : public mc_control::MCController {
	
    public:

	DoorController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

	virtual bool run() override;
	virtual void reset(const mc_control::ControllerResetData & reset_data) override;

	std::shared_ptr<mc_tasks::CoMTask> comTask;
	std::shared_ptr<mc_tasks::OrientationTask> orBodyTask;
	std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
	std::shared_ptr<mc_tasks::ComplianceTask> compTask;

	std::shared_ptr<tasks::qp::PostureTask> doorPostureTask;
	
	mc_rbdyn::ForceSensorsCalibrator calibrator;

	mc_solver::KinematicsConstraint doorKinematicsConstraint;
	mc_solver::CollisionsConstraint doorCollisionsConstraint;

	bool read_write_msg(std::string & in, std::string & out) override;

	DoorTaskState * state = nullptr;
	Eigen::Vector3d knob_pos;
    };
}
