/*
 *
*/
#include <stdlib.h>
#include <gazebo/msgs/msgs.hh>
#include "robot_gui.hh"
using namespace gazebo;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;

//? extern bool camera_off;
//? namespace cv {
//? extern void getImage(ConstImageStampedPtr &_img);
//? }
bool camera_off = true;

//-----------------------------
namespace cv {

void getImage(ConstImageStampedPtr &_img)
{

	if (camera_off) return;
	static int count = -1; count++;
	std::cout << "getImage: count="<<count<<std::endl;std::cout.flush();//-
//+	int
//+		width = _img->image().width(),
//+		height = _img->image().height(),
//+		size = _img->image().data().length();
//+
//+  std::cout << ">>>>> Image "<< count << ": " << width << " x " << height << ", size= " << size <<", format: " << _img->image().pixel_format() << "<<<<<<<<\n" << std::endl; std::cout.flush();
//+
//+	char *img = new char[size + 1];
//+	memcpy(img, _img->image().data().c_str(), size);
//+	Mat image(height, width, CV_8UC3, img);
//+	imshow("camera", image);
//+	
//+	waitKey(0);
//+	delete img;

	camera_off = true;

}
} // end namespace cv


// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(VehicleGUI)

/////////////////////////////////////////////////
VehicleGUI::VehicleGUI() : 
	GUIPlugin() 
{
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Spawn Sphere"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(120, 30);

  // Create a node for camera
  this->camera = transport::NodePtr(new transport::Node());
  this->camera->Init();
//?  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
	// Subscribe to camera
	std::string cam_topic = std::string("/gazebo/default/Bot620-2017") 
//+ std::string(MODEL_NAME)  
+ std::string("/") + std::string(CAMERA_TOPIC);
	transport::SubscriberPtr sub = camera->Subscribe(cam_topic, getImage);
	std::cout << "Subscribed to camera topic: " << cam_topic << std::endl;

	//VEHICLE PLUGIN:
  this->drive = transport::NodePtr(new transport::Node());
  this->drive->Init();
	std::string topic_name = std::string("~/") + std::string(MODEL_NAME) + std::string("/") + std::string(CMD_TOPIC);
	std::cout << "VehicleGUI::VehicleGUI():Advertise topic_name="<<topic_name<<std::endl;std::cout.flush();
  this->controlPub = this->drive->Advertise<gazebo::msgs::Pose>(topic_name);
	
//?  this->stateSub = this->drive->Subscribe<msgs::VehicleGUI>("~/pioneer/state", &VehicleGUI::OnState, this);


  // Connect hotkeys.
  QShortcut *increaseGasKey = new QShortcut(QKeySequence("w"), this);
  QObject::connect(increaseGasKey, SIGNAL(activated()), this, SLOT(OnIncreaseGas()));

  QShortcut *increaseGasArrow = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(increaseGasArrow, SIGNAL(activated()), this, SLOT(OnIncreaseGas()));

  QShortcut *decreaseGasKey = new QShortcut(QKeySequence("s"), this);
  QObject::connect(decreaseGasKey, SIGNAL(activated()), this, SLOT(OnDecreaseGas()));

  QShortcut *decreaseGasArrow = new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(decreaseGasArrow, SIGNAL(activated()), this, SLOT(OnDecreaseGas()));

  QShortcut *turnLeftKey = new QShortcut(QKeySequence("a"), this);
  QObject::connect(turnLeftKey, SIGNAL(activated()), this, SLOT(OnTurnLeft()));
  QShortcut *turnLeftArrow = new QShortcut(QKeySequence(Qt::Key_Left), this);
  QObject::connect(turnLeftArrow, SIGNAL(activated()), this, SLOT(OnTurnLeft()));

  QShortcut *turnRightKey = new QShortcut(QKeySequence("d"), this);
  QObject::connect(turnRightKey, SIGNAL(activated()), this, SLOT(OnTurnRight()));
  QShortcut *turnRightArrow = new QShortcut(QKeySequence(Qt::Key_Right), this);
  QObject::connect(turnRightArrow, SIGNAL(activated()), this, SLOT(OnTurnRight()));

  QShortcut *goStraightKey = new QShortcut(QKeySequence("x"), this); 
	QObject::connect(goStraightKey, SIGNAL(activated()), this, SLOT(OnGoStraight()));
  QShortcut *goStraightArrow = new QShortcut(QKeySequence(Qt::Key_Home), this); 
	QObject::connect(goStraightArrow, SIGNAL(activated()), this, SLOT(OnGoStraight()));

  QShortcut *stopKey = new QShortcut(QKeySequence("z"), this);
  QObject::connect(stopKey, SIGNAL(activated()), this, SLOT(OnStop()));
  QShortcut *stopArrow = new QShortcut(QKeySequence(Qt::Key_End), this);
  QObject::connect(stopArrow, SIGNAL(activated()), this, SLOT(OnStop()));

  QShortcut *slowDrive = new QShortcut(QKeySequence("1"), this);
  QObject::connect(slowDrive, SIGNAL(activated()), this, SLOT(OnSlowDrive()));

  QShortcut *normalDrive = new QShortcut(QKeySequence("2"), this);
  QObject::connect(normalDrive, SIGNAL(activated()), this, SLOT(OnNormalDrive()));

  QShortcut *fastDrive = new QShortcut(QKeySequence("3"), this);
  QObject::connect(fastDrive, SIGNAL(activated()), this, SLOT(OnFastDrive()));

  QShortcut *autoDrive = new QShortcut(QKeySequence("0"), this);
  QObject::connect(autoDrive, SIGNAL(activated()), this, SLOT(OnAutoDrive()));

}

/////////////////////////////////////////////////
VehicleGUI::~VehicleGUI()
{
}


void VehicleGUI::Send() {
//	std::cout<<"Send: velocity="<<velocity<<", angle="<<angle<<std::endl;//-
	ignition::math::Pose3d pose(velocity, 0.0, 0.0, 0.0, 0.0, DEG2RAD(angle));
	gazebo::msgs::Pose pose_msg;
	gazebo::msgs::Set(&pose_msg, pose);//generate message

	controlPub->Publish(pose_msg); //publish message
	gazebo::common::Time::MSleep(20);//sleep ?
}

void VehicleGUI::OnIncreaseGas(){
	velocity += velocity_inc;
	Send();
}
void VehicleGUI::OnDecreaseGas(){
	velocity -= velocity_inc;
	Send();
}
void VehicleGUI::OnStop(){
	velocity = 0.0;
	Send();
}
void VehicleGUI::OnTurnLeft(){
	if (angle > -angle_max) {
		angle -= angle_inc;
	} else {
		angle = -angle_max;
	}
	Send();
}
void VehicleGUI::OnTurnRight(){
	if (angle < angle_max) {
		angle += angle_inc;
	} else {
		angle = angle_max;
	}
	Send();
}
void VehicleGUI::OnGoStraight(){
	angle = 0.0;
	Send();
}
void VehicleGUI::OnSlowDrive(){
	velocity_inc = 0.1;
	angle_inc = 1.0;
}
void VehicleGUI::OnNormalDrive(){
	velocity_inc = 0.2;
	angle_inc = 2.0;
}
void VehicleGUI::OnFastDrive(){
	velocity_inc = 0.5;
	angle_inc = 5.0;
}
void VehicleGUI::OnAutoDrive(){
	int attempts = 0, max_attempts = 1000000;
	std::cout << "START Autonomous" << std::endl;
//-	std::ifstream img_file(std::string(IMAGE_FILE));
//-  if(!img_file.fail()){ }
	camera_off = false;
	for (; camera_off == false && attempts<max_attempts; attempts++) {
		std::cout << "Waiting on camera " << attempts+1 << " of " << max_attempts << '\r'; std::cout.flush();
//?		sleep(1);
	} 
	std::cout << std::endl;
	if (attempts == max_attempts) {
		std::cout << "Camera Stopped Working!" << std::endl;
	}
	camera_off = true;
	std::cout << "END Autonomous" << std::endl;
}



/////////////////////////////////////////////////

void VehicleGUI::OnButton()
{
  msgs::Model model;
  model.set_name("plugin_unit_sphere_" + std::to_string(this->counter++));
  msgs::Set(model.mutable_pose(), ignition::math::Pose3d(0, 0, 1.5, 0, 0, 0));
  const double mass = 1.0;
  const double radius = 0.5;
  msgs::AddSphereLink(model, mass, radius);

  std::ostringstream newModelStr;
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

  // Send the model to the gazebo server
  msgs::Factory msg;
  msg.set_sdf(newModelStr.str());
//?  this->factoryPub->Publish(msg);
}

