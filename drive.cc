// https://github.com/jaejunlee0538/gazebo_personal_tutorial/tree/master/move_pioneer2dx
// http://www.cplusplus.com/forum/general/16335/
// http://stackoverflow.com/questions/448944/c-non-blocking-keyboard-input

//#include <conio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <ctime>
#include <termios.h>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "robot.h"

//
// Global variables
//
double 
	velocity = 0.0,
	angle = 0.0;

using namespace cv;

namespace cv {
extern void navigate(ConstImageStampedPtr &_img);
}

namespace Time {
	std::time_t stop_time;
	void start(std::time_t duration) {
		stop_time = std::time(nullptr) + duration;
	}
	std::time_t time_left() {
		return stop_time - std::time(nullptr);
	}
	bool timeout() {
		return std::time(nullptr) > stop_time;
	}
	void wait(int n) {
		gazebo::common::Time::MSleep(n);	
	}
	void pause() {
		wait(100);
	}
}


struct termios orig_termios;

void reset_terminal_mode()
{
	tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
	struct termios new_termios;

	/* take two copies - one for now, one for later */
	tcgetattr(0, &orig_termios);
	memcpy(&new_termios, &orig_termios, sizeof(new_termios));

	/* register cleanup handler, and set the new terminal mode */
	atexit(reset_terminal_mode);
	cfmakeraw(&new_termios);
	tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
	struct timeval tv = { 0L, 0L };
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(0, &fds);
	return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
	int r;
	unsigned char c;
	if ((r = read(0, &c, sizeof(c))) < 0) {
		return r;
	} else {
		return c;
	}
}

#include <iostream>

// Get Input
bool getInput(char *c)
{
	if (kbhit())
	{
		 *c = getch();
		 return true; // Key Was Hit
	}
	return false; // No keys were pressed
}


#define DEG2RAD(x)  x*0.017453293
#define RAD2DEG(x)  x*57.29577951

void drive(gazebo::transport::PublisherPtr robot) {

		ignition::math::Pose3d pose(-velocity, 0.0, 0.0, 0.0, 0.0, DEG2RAD(angle));
		gazebo::msgs::Pose pose_msg;
		gazebo::msgs::Set(&pose_msg, pose);//generate message
		robot->Publish(pose_msg); //publish message
}

void stop(gazebo::transport::PublisherPtr robot) {
	velocity = angle = 0.0;
	drive(robot);
}

// 
// Autonomous navigation
//
void autonomous(gazebo::transport::PublisherPtr robot) {
	std::cout << "START Autonomous\n\r";

	// Go blindly to position:
	velocity = BLIND_SPEED; angle = 0.0;
	Time::start(BLIND_TIME);
	drive(robot);
	while (!Time::timeout()) {
		Time::pause();
	}
	stop(robot);

	// 
	// Navigate by camera
	//
	static int max_attempts = 100;
	extern bool 
		reached_target,
		camera_off;
	Time::start(VISUAL_TIME);
	while (!reached_target && !Time::timeout()) {
		int attempts = 0;
		camera_off = false;
		for (; camera_off == false && attempts<max_attempts; attempts++) {
			// It is assumed that the global variable
			// 'camera_off' is updated in the callback 
			// function 'navigate(...)'
			Time::pause();
		} 
		if (attempts == max_attempts) {
			std::cout << "Camera failed after "<<attempts<<" attempts!\n\r";
			break;
		}
		camera_off = true;
		std::cout << "Time left: "<<Time::time_left()<<"\r"; std::cout.flush();
		drive(robot);
	}
	stop(robot);
	//TODO: Here you do gear deposition
	//...
	std::cout << "\n\rEND Autonomous\n\r";std::cout.flush();
}

int main(int argc, char** argv){

	gazebo::client::setup(argc, argv);

	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	/*
		DiffDrivePlugin is subscribing ~/${model_name}/vel_cmd
		and calculates left/right wheel speed based on vel_cmd message.
	*/
	std::string cmd_topic 
	= std::string("~/")
	+ std::string(MODEL_NAME)
	+ std::string("/")
	+ std::string(CMD_TOPIC);

	gazebo::transport::PublisherPtr robot = node->Advertise<gazebo::msgs::Pose>(cmd_topic);

	robot->WaitForConnection();
	std::cout << "Publishing to '" << cmd_topic << std::endl; std::cout.flush();

	gazebo::transport::NodePtr camera_stream(new gazebo::transport::Node());
	camera_stream->Init();

	std::string camera_topic 
	= std::string("/gazebo/default/Bot620-2017") 
	+ std::string("/") 
	+ std::string(CAMERA_TOPIC);

	gazebo::transport::SubscriberPtr sub = camera_stream->Subscribe(camera_topic, navigate);


	// 
	// Process keyboard input
	//
	set_conio_terminal_mode();
	char key = ' '; 

	while(key != 'q')
	{
//		std::cout << "Vel: "<<velocity<<", angle: " << angle << "\r"; std::cout.flush();
		while(!getInput(&key)) {
		}
		drive(robot);
		Time::pause();
		switch (key) {
		case '+':
			velocity += 0.1;
			break;
		case '-':
			velocity -= 0.1;
			break;
		case '0':
			velocity = 0.0;
			break;
		case 'l':
		case 'L':
			angle -= 2.0;
			break;
		case 'r':
		case 'R':
			angle += 2.0;
			break;
		case 's':
		case 'S':
			angle = 0.0;
			velocity = 0.0;
			break;
		case 'a':
		case 'A':
			autonomous(robot);	
			break;
		} 
	}
//	std::cout<<"\n";
	gazebo::shutdown();
}

