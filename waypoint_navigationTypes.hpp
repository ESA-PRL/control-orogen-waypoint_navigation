#pragma once

namespace waypoint_navigation
{

struct pathTrackerConfig
{
    // Minimum radius of robot's Ackermann turn			 (m)
    double minTurnRadius;

    // Robot's translational velocity during Ackermann turns	 (m/s)
    double translationalVelocity;

    // Robot's rotational velocity during point turns		 (rad/s)
    double rotationalVelocity;

    // Safety corridor (1/2 width) arround the nominal trajectory	 (m)
    double corridor;

    // Maximum allowed disalignment from target heading in the final pose (rad)
    double maxDisalignment;

    // Distance to the lookahead point of the Pure Pursuit Algorithm (m)
    double lookaheadDistance;

    // Is backward motion permitted? true/false
    bool backwards;
};

struct controllerPDConfig
{
    double P;
    double D;
    double saturation;  // Input disalignment error saturation
};

}  // namespace waypoint_navigation