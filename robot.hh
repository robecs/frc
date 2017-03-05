/*
 *
*/
#ifndef _GAZEBO_VEHICLE_PLUGIN_HH_
#define _GAZEBO_VEHICLE_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"
#include "robot.h"

namespace gazebo
{
  class GAZEBO_VISIBLE Robot : public ModelPlugin
  {
    public: Robot();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void OnVelMsg(ConstPosePtr &_msg);

    private: transport::NodePtr node;
    private: transport::SubscriberPtr velSub;

    private: physics::ModelPtr model;
    private: physics::JointPtr frontLeftJoint, frontRightJoint, backLeftJoint, backRightJoint;
    private: event::ConnectionPtr updateConnection;
    private: double wheelSpeed[2];
    private: double wheelSeparation;
    private: double wheelRadius;
    private: common::Time prevUpdateTime;
		double 
			speed_multiplier = 1.0,
			angle_multiplier = 1.0;

    private: physics::LinkPtr link, leftWheelLink, rightWheelLink;
  };
}
#endif
