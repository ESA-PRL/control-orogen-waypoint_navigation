/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class encapsulating the pure-pursuit based path
 * follower, which was implemented for the six-wheeled ExoTeR rover.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Filip, email:jan.filip@esa.int, jan.filip2@gmail.com
 * Supervised by: Martin Azkarate, email:martin.azkarate@esa.int
 *
 * Date of creation: Dec 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */


#include "Task.hpp"
#include <WaypointNavigation.hpp>

using namespace waypoint_navigation;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    pathTracker = NULL;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
	if (! TaskBase::configureHook())
      		return false;
    bool configSuccessful;

	ptConfig = _ptConfig.value();
	pathTracker = new waypoint_navigation_lib::WaypointNavigation();
    configSuccessful = pathTracker->configure(
		ptConfig.minTurnRadius,
		ptConfig.translationalVelocity,
		ptConfig.rotationalVelocity,
		ptConfig.corridor,
		ptConfig.lookaheadDistance,
        ptConfig.backwards);

    controllerPDConfig pd = _pdConfig.value();
    configSuccessful &= pathTracker->configurePD(pd.P, pd.D, pd.saturation);
    configSuccessful &= pathTracker->configureTol(_tolPos.value(), _tolHeading.value()*M_PI/180.0);

    positionValid = false;
	roverStopped = false;
	trajectory.clear();

    mc_prev.translation = 0; mc_prev.rotation = 0;

  return true;
}

void Task::updateHook()
{
    // -------------------  TRAJECTORY SETTING   ---------------
    //if(_trajectory.readNewest(trajectory) == RTT::NewData ) { // Trajectory input contains new data
    if(_trajectory.read(trajectory) == RTT::NewData ) {
        //convert to driver format
        //std::cout << "WaypointNavigation::updateHook(), Task has  " << trajectory.size() << " points in trajectory." << std::endl;

		// Pass the waypoints to the library using pointers
        std::vector<base::Waypoint*> waypoints;
        for (std::vector<base::Waypoint>::const_iterator it = trajectory.begin();
                it != trajectory.end(); ++it) // Iterate through trajectory received: [1st to Nth].
        {
            waypoints.push_back(new base::Waypoint(*it)); // Add element to the end of the vector
        }
		pathTracker->setTrajectory(waypoints);
        //std::cout << "WaypointNavigation::updateHook(), Trajectory set to path tracker." << std::endl;
    }

    // -------------------   NEW POSE READING   ----------------
    base::samples::RigidBodyState pose;
//    if(_pose.readNewest(pose) != RTT::NoData )
    if(_pose.read(pose) != RTT::NoData )
    {
        positionValid = pathTracker->setPose(pose);
    }

    // -------------------   MOTION UPDATE      ----------------
    // Create zero motion command
    base::commands::Motion2D mc;
    mc.translation = 0.0; mc.rotation = 0.0;
    if(!trajectory.empty() && positionValid)
    {
        // If position data are valid, calculate the motion command
        pathTracker->update(mc);
        // Write lookahead point to the output (only if there is the trajectory)
        _currentWaypoint.write(*(pathTracker->getLookaheadPoint()));
    }

    // -------------- SPEED ADJUSTMENT -------------------------
    if(_speed_input.connected())
    {
        double new_speed;
        if(_speed_input.read(new_speed) == RTT::NewData)
        {
            bool configSuccessful;
            ptConfig.translationalVelocity = new_speed;
            configSuccessful = pathTracker->configure(
                    ptConfig.minTurnRadius,
                    ptConfig.translationalVelocity,
                    ptConfig.rotationalVelocity,
                    ptConfig.corridor,
                    ptConfig.lookaheadDistance,
                    ptConfig.backwards);
        }
    }

    //-------------- State Update from the library to the component
    waypoint_navigation_lib::NavigationState currentState = pathTracker->getNavigationState();
    switch(currentState) {
            case waypoint_navigation_lib::DRIVING:{
                roverStopped = false;
                state(DRIVING);
                break;
            }
            case waypoint_navigation_lib::ALIGNING:{
                state(ALIGNING);
                roverStopped = false;
                break;
            }
            case waypoint_navigation_lib::TARGET_REACHED:{
                state(TARGET_REACHED);
                this->stopRover();
                //mc.translation = 0.00001; // This command puts all wheel straight when reaching the target. Usually would stay in Point Turn configuration.
                break;
            }
            case waypoint_navigation_lib::OUT_OF_BOUNDARIES:{
                state(OUT_OF_BOUNDARIES);
                this->stopRover();
                break;
            }
            case waypoint_navigation_lib::NO_TRAJECTORY:{
                state(NO_TRAJECTORY);
                this->stopRover();
                break;
            }
            case waypoint_navigation_lib::NO_POSE:{
                state(NO_POSE);
                this->stopRover();
                break;
            }
            default:{
				// safety first!
                this->stopRover();
                break;
            }
        }
    _trajectory_status.write((int)currentState);
    _current_segment.write(pathTracker->getCurrentSegment());
    // Write motion command to the ouput if different from previous
    if(( _repeatCommand.value() || mc.translation != mc_prev.translation || mc.rotation != mc_prev.rotation) && !roverStopped ){
        _motion_command.write(mc);
        mc_prev = mc;
    }

}

void Task::errorHook(){
	stopRover();
}

void Task::stopHook(){
	stopRover();
}


void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    delete pathTracker;
}

void Task::stopRover(){
    roverStopped = true;
	base::commands::Motion2D mc;
    mc.translation = 0.0; mc.rotation = 0.0;
    if( mc.translation != mc_prev.translation ||
    	mc.rotation    != mc_prev.rotation )
    {
        _motion_command.write(mc);
        mc_prev = mc;
    }
}
