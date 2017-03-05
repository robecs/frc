/*
 *
*/

#include <iostream>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "robot.hh"


//#define VELOCITY_CONTROL

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Robot)

enum {RIGHT, LEFT};

/////////////////////////////////////////////////
Robot::Robot()
{
  this->wheelSpeed[LEFT] = this->wheelSpeed[RIGHT] = 0;
  this->wheelSeparation = 1.0;
  this->wheelRadius = 1.0;
}

/////////////////////////////////////////////////
void Robot::Load(
	physics::ModelPtr _model,
	sdf::ElementPtr _sdf
)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

//-	std::string topic_name = std::string("~/") + this->model->GetName() + std::string("/") + std::string(CMD_TOPIC);
	std::string topic_name = std::string("~/")+std::string(MODEL_NAME)+std::string("/")+std::string(CMD_TOPIC);
  this->velSub = this->node->Subscribe(topic_name, &Robot::OnVelMsg, this);
	std::cout << "robot.cc:Subscribed to topic: '" << topic_name << std::endl;

  if (_sdf->HasElement("speed_multiplier")) {
    speed_multiplier = _sdf->Get<double>("speed_multiplier");
  } 
  if (_sdf->HasElement("angle_multiplier")) {
    angle_multiplier = _sdf->Get<double>("angle_multiplier");
  } 
//  std::cout << "robot.cc:Initializing motor: " << topic_name << " speed/angle multipliers: " << speed_multiplier << ", " << angle_multiplier << std::endl;

  if (!_sdf->HasElement("front_left_joint"))
    gzerr << "Robot plugin missing <front_left_joint> element\n";

  if (!_sdf->HasElement("front_right_joint"))
    gzerr << "Robot plugin missing <front_right_joint> element\n";

  this->frontLeftJoint = _model->GetJoint(
      _sdf->GetElement("front_left_joint")->Get<std::string>());
  this->frontRightJoint = _model->GetJoint(
      _sdf->GetElement("front_right_joint")->Get<std::string>());

  if (!this->frontLeftJoint)
    gzerr << "Unable to find front left joint["
          << _sdf->GetElement("front_left_joint")->Get<std::string>() << "]\n";
  if (!this->frontRightJoint)
    gzerr << "Unable to find front right wheel joint["
          << _sdf->GetElement("front_right_joint")->Get<std::string>() << "]\n";

  if (!_sdf->HasElement("back_left_joint"))
    gzerr << "Robot plugin missing <back_left_joint> element\n";

  if (!_sdf->HasElement("back_right_joint"))
    gzerr << "Robot plugin missing <back_right_joint> element\n";

  this->backLeftJoint = _model->GetJoint(
      _sdf->GetElement("back_left_joint")->Get<std::string>());
  this->backRightJoint = _model->GetJoint(
      _sdf->GetElement("back_right_joint")->Get<std::string>());

  if (!this->backLeftJoint)
    gzerr << "Unable to find back left joint["
          << _sdf->GetElement("back_left_joint")->Get<std::string>() << "]\n";
  if (!this->backRightJoint)
    gzerr << "Unable to find back right wheel joint["
          << _sdf->GetElement("back_right_joint")->Get<std::string>() << "]\n";

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Robot::OnUpdate, this));
std::cout << "robot.cc:Load:END\n";//-
}

/////////////////////////////////////////////////
void Robot::Init()
{
std::cout<<"Robot::Init:BEGIN\n";std::cout.flush();//-
  this->wheelSeparation = this->frontLeftJoint->GetAnchor(0).Distance(
      this->frontRightJoint->GetAnchor(0));

  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(this->frontLeftJoint->GetChild());

  math::Box bb = parent->GetBoundingBox();
  // This assumes that the largest dimension of the wheel is the diameter
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;
std::cout<<"wheelSeparation=" << wheelSeparation << ", weelRadius=" << wheelRadius << std::endl;std::cout.flush();//-
std::cout<<"Robot::Init:END\n";std::cout.flush();//-
}

/////////////////////////////////////////////////
void Robot::OnVelMsg(ConstPosePtr &_msg)
{
  double 
  	v = speed_multiplier * _msg->position().x(),
	  a = angle_multiplier *  msgs::ConvertIgn(_msg->orientation()).Euler().Z();

//-std::cout << "OnVelMsg: vl="<<v+a*wheelSeparation<<", vr="<<v-a*wheelSeparation<<std::endl;std::cout.flush();//-
  this->wheelSpeed[LEFT] = v + a * wheelSeparation; // / 2.0;
  this->wheelSpeed[RIGHT] = v - a * wheelSeparation; // / 2.0;
}

/////////////////////////////////////////////////
void Robot::OnUpdate()
{
  /* double d1, d2;
  double dr, da;

  this->prevUpdateTime = currTime;

  // Distance travelled by front wheels
  d1 = stepTime.Double() * this->wheelRadius * this->frontLeftJoint->GetVelocity(0);
  d2 = stepTime.Double() * this->wheelRadius * this->frontRightJoint->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / this->wheelSeparation;
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  */

#ifdef VELOCITY_CONTROL
  double 
		leftVelDesired = (this->wheelSpeed[LEFT] / this->wheelRadius),
		rightVelDesired = (this->wheelSpeed[RIGHT] / this->wheelRadius);
//-std::cout<<"Time="<<currTime<<": Vel=("<<this->frontLeftJoint->GetVelocity(0)<<','<<this->frontRightJoint->GetVelocity(0)<<", DesVel=("<<leftVelDesired<<','<<rightVelDesired<<")"<<std::endl;std::cout.flush();//-

  this->frontLeftJoint->SetVelocity(0, leftVelDesired);
  this->backLeftJoint->SetVelocity(0, leftVelDesired);

  this->frontRightJoint->SetVelocity(0, rightVelDesired);
  this->backRightJoint->SetVelocity(0, rightVelDesired);

#else
	double	
		leftForce = wheelSpeed[LEFT],
		rightForce = wheelSpeed[RIGHT];

  this->frontLeftJoint->SetForce(0, leftForce);
  this->backLeftJoint->SetForce(0, leftForce);

  this->frontRightJoint->SetForce(0, rightForce);
  this->backRightJoint->SetForce(0, rightForce);

#endif
}
