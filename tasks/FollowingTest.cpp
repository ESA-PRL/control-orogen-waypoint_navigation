/****************************************************************
 *
 * Copyright (c) 2016
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Description: Class for simple simulation (only integration of
 * velocity commands) of robot motions.
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

#include "FollowingTest.hpp"

using namespace waypoint_navigation;

FollowingTest::FollowingTest(std::string const& name)
    : FollowingTestBase(name), mStartPose(), mStartPoseReceived(false), mTimeStart()
{
}

FollowingTest::FollowingTest(std::string const& name, RTT::ExecutionEngine* engine)
    : FollowingTestBase(name, engine), mStartPose(), mStartPoseReceived(false), mTimeStart()
{
}

FollowingTest::~FollowingTest() {}

bool FollowingTest::configureHook()
{
    if (!FollowingTestBase::configureHook()) return false;
    // Set up robot pose based on config values
    base::Vector2d position2d = _start_position.value();

    mStartPose.position = Eigen::Vector3d(position2d(0), position2d(1), 0);
    mStartPose.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(_start_heading.value() / 180.0 * M_PI, Eigen::Vector3d::UnitZ()));

    mStartPoseReceived = true;
    mCurrentPose = mStartPose;
    std::cout << "Config of robot pose done!" << std::endl;
    std::cout << "Robot = (" << mCurrentPose.position.x() << "," << mCurrentPose.position.y() << ","
              << mCurrentPose.position.z() << "), "
              << "yaw = " << mCurrentPose.getYaw() * 180 / M_PI << " deg." << std::endl;
    return true;
}

bool FollowingTest::startHook()
{
    std::cout << "FollowingTask::startHook() called. " << std::endl;
    if (!FollowingTestBase::startHook())
    {
        return false;
    }
    // _robot_pose.write(mCurrentPose);
    mTimeStart = base::Time::now();
    std::cout << "FollowingTask::startHook() t = " << mTimeStart.toSeconds() << std::endl;
    return true;
}

void FollowingTest::updateHook()
{
    FollowingTestBase::updateHook();
    // MOTION SIMULATION HERE

    if (!mStartPoseReceived)
    {
        return;
        std::cout << "FollowingTask::updateHook() start pose not received." << std::endl;
    }
    base::samples::RigidBodyState newPose;
    int rttPortState = _start_pose.readNewest(newPose);
    if (rttPortState == RTT::NewData && rttPortState != RTT::NoData)
    {
        mCurrentPose = newPose;
        std::cout << "FollowingTest::updateHook(): new pose received, written out." << std::endl;
        _robot_pose.write(mCurrentPose);
        return;  // No new motion simulation
    }

    // READ NEWEST MOTION COMMAND
    base::commands::Motion2D mc_new;
    if (_motion_command.readNewest(mc_new) == RTT::NewData)
    {
        mc = mc_new;
    }

    // EXECUTE NEWEST MOTION COMMAND
    base::Time time_now = base::Time::now();
    double time_sec = (time_now - mTimeStart).toSeconds();

    // Avoids the big initial cap.
    if (time_sec > 2)
    {
        mTimeStart = base::Time::now();
        time_sec = 0;
    }

    // Get time difference dt for simulation
    mTimeStart = time_now;
    double dt = time_sec;
    ;

    std::cout << "FollowingTask::updateHook() simulating the motion with dt = " << dt << std::endl;

    // SIMULATE MOTION COMMAND
    double yaw = mCurrentPose.getYaw();
    Eigen::AngleAxisd toWCF, robotRot;
    toWCF = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    if (fabs(mc.translation) < 0.000001)
    {
        // Point turn
        robotRot = Eigen::AngleAxisd(mc.rotation * dt, Eigen::Vector3d::UnitZ());
        mCurrentPose.orientation = Eigen::Quaterniond(robotRot) * mCurrentPose.orientation;
    }
    else if (fabs(mc.rotation) < 0.000001)
    {
        // Straight line
        mCurrentPose.position += (mc.translation * dt) * (toWCF * Eigen::Vector3d::UnitX());
    }
    else
    {
        // Ackermann
        Eigen::Vector3d turnCenter;
        turnCenter << 0.0, mc.translation / mc.rotation, 0.0;
        turnCenter = toWCF * (turnCenter) + mCurrentPose.position;
        robotRot = Eigen::AngleAxisd(mc.rotation * dt, Eigen::Vector3d::UnitZ());
        mCurrentPose.position = robotRot * (mCurrentPose.position - turnCenter) + turnCenter;
        mCurrentPose.orientation = Eigen::Quaterniond(robotRot) * mCurrentPose.orientation;
    }

    _robot_pose.write(mCurrentPose);
}

void FollowingTest::errorHook() { FollowingTestBase::errorHook(); }
void FollowingTest::stopHook() { FollowingTestBase::stopHook(); }
void FollowingTest::cleanupHook() { FollowingTestBase::cleanupHook(); }
