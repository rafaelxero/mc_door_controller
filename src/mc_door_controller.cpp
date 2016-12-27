#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/logging.h>
#include "mc_door_controller.h"
#include "mc_door_fsm.h"

namespace mc_door {

    DoorController::DoorController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
	: MCController({robot_module,
                    mc_rbdyn::RobotLoader::get_robot_module("env", std::string("/home/rcisneros/src/jrl/catkin_ws/src/mc_ros/mc_int_obj_description"), std::string("drcfinals_door")),
                    mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt),
	  calibrator(robot_module),
	  doorKinematicsConstraint(robots(), 1, dt) {

	solver().addConstraintSet(contactConstraint);
	solver().addConstraintSet(dynamicsConstraint);
	solver().addConstraintSet(doorKinematicsConstraint);
	solver().addConstraintSet(selfCollisionConstraint);

	doorCollisionsConstraint = mc_solver::CollisionsConstraint(robots(), 0, 1, 1.0);
	doorCollisionsConstraint.addCollisions(solver(), {
		{"LARM_LINK6", "door", 0.05, 0.02, 0.0}});
	solver().addConstraintSet(doorCollisionsConstraint);

	solver().addTask(postureTask.get());

	solver().setContacts({
		{robots(), 0, 2, "LFullSole", "AllGround"},
		{robots(), 0, 2, "RFullSole", "AllGround"}
	    });

	auto & door = robots().robots()[1];

	door.mbc().zero(door.mb());
	rbd::forwardKinematics(door.mb(), door.mbc());	

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
	auto& hrp2 = robots().robots()[0];
	hrp2.mbc().q[0] = {1, 0, 0, 0, -0.75, 0.4, 0.773};
	rbd::forwardKinematics(hrp2.mb(), hrp2.mbc());

	state = new IniState();
    }

    bool DoorController::read_write_msg(std::string& in, std::string& out)
    {
	std::stringstream ss(in);
	std::string token;
	ss >> token;
	if (token == "set_knob_pos") {
	    double x, y, z;
            ss >> x;
	    ss >> y;
	    ss >> z;
	    if (ss.fail()) {
	        out = "Unable to set knob pos";
		return false;
	    }
	    else {
		knob_pos = Eigen::Vector3d(x, y, z);
		out = "Knob pos set!";
		return true;
	    }
	}
	return false;
    }

    SIMPLE_CONTROLLER_CONSTRUCTOR("Door", DoorController)
}
