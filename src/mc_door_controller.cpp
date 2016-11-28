#include <mc_rtc/logging.h>
#include "mc_door_controller.h"
#include "mc_door_fsm.h"

namespace mc_door {

    DoorController::DoorController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: mc_control::MCController(robot_module, dt) {
	
	solver().addConstraintSet(contactConstraint);
	solver().addConstraintSet(kinematicsConstraint);
	solver().addTask(postureTask.get());
	solver().setContacts({});

	LOG_SUCCESS("DoorController init done " << this);
    }

    bool DoorController::run() {

	bool ret = MCController::run();

	if (ret) {

	    if (state) {

		auto nstate = state->update(*this);
		if (nstate != state) {
		    
		    LOG_SUCCESS("Completed " << state->name)

		    delete state;
		    state = nstate;

		    if (state == nullptr) {
			LOG_SUCCESS("Completed FSM")
		    }
		}
	    }
	}

	return ret;
    }

    void DoorController::reset(const mc_control::ControllerResetData& reset_data) {

	MCController::reset(reset_data);

	state = new IniState();
    }

    SIMPLE_CONTROLLER_CONSTRUCTOR("Door", DoorController)
}
