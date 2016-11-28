#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>

namespace mc_door {

    struct DoorTaskState;

    struct DoorController : public mc_control::MCController {
	
    public:

	DoorController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

	virtual bool run() override;
	virtual void reset(const mc_control::ControllerResetData& reset_data) override;

	std::shared_ptr<mc_tasks::CoMTask> comTask;
	std::shared_ptr<mc_tasks::EndEffectorTask> efTask;

	DoorTaskState * state = nullptr;
    };
}
