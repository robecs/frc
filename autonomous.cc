#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
using namespace gazebo;
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;
#include "robot.h"

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> int sgn(T val) {
	return val < 0 ? -1 : 1;
}

extern double velocity, angle;

bool
	camera_off = true,
	reached_target = false;


//-----------------------------
namespace cv {
//
// Navigate by camera
//
void navigate(ConstImageStampedPtr &_img)
{

	if (camera_off) return;
	static int 
		half_nbuf = 4, //2*nbuf = number of pixels for averaging 
		nbuf = 2*half_nbuf;

	// 
	// Acquire the image
	//
	int
		width = _img->image().width(),
		height = _img->image().height(),
		size = _img->image().data().length(),
		desired_target_position = width/2, // middle of the screen
		desired_target_size = DESIRED_TARGET_IMAGE_SIZE, // in pixels
		min_offset = nbuf,
		min_size_mismatch = half_nbuf,
		xmin = nbuf,
		xmax = width - nbuf,
		ymin = 0.2 * height,
		ymax = 0.8 * height;

	char *img = new char[size + 1];
	memcpy(img, _img->image().data().c_str(), size);
	Mat image(height, width, CV_8UC3, img);
	//
	// Analyze the image and navigate to the target
	//
	static float turning_angle = 8.0; // degrees
	int 
		m1 = 0, x1a = 0, // average x-coordinates of object 1
		m2 = 0, x2a = 0; // average x-coordinates of object 2
	//
	// Read image by pixels
	//
	for (int y=ymin; y<ymax; y++) {
		int 
			n1 = 0, x1 = -1,
			n2 = 0, x2 = -1,
			state = 0; 
				// 0 - before object 1
				// 1 - in object 1
				// 2 - after object 1
				// 3 - in object 2
				// 4 - after object 2
		for (int x=xmin; x<xmax; x++) {
			float ColorIntensity[] = {0,0,0}; 
			// [0] = red, [1] = green, [2] = blue

			// To provide resistance from random noise 
			// apply a filter by simple averaging of color 
			// intensity over a pixel strip of length 'nbuf'
			for (int i=x-half_nbuf; i<x+half_nbuf; i++) {
				Vec3b intensity = image.at<Vec3b>(y, i);
				for (int j=0; j<3; j++) 
					ColorIntensity[2-j] += (float)intensity.val[j];
			}

			// Compute the relative intensity of the target color
			// this avoid the effects of variable lighting and shades
			float target_light_intensity = 0.0;
			for (int j=0; j<3; j++) {
				float normalized_intensity = ColorIntensity[j]/nbuf;
				target_light_intensity += normalized_intensity;
				ColorIntensity[j] = normalized_intensity;
			}
			float target_color_contrast // relative color intensity
			= ColorIntensity[TARGET_COLOR] / target_light_intensity;
			if (target_color_contrast > TARGET_COLOR_CONTRAST) {
				// 
				// Determine if both objects are in full view
				// i.e. state==4 -> both objects are in full view
				switch (state) {
				case 0: // first time we saw this color during x-scan
					x1 = x; n1 = 1;
					state = 1;
					break;
				case 1: // we are inside object 1
					x1 += x; n1++;
					break;
				case 2: // we are past object 1
					x2 = x; n2 = 1;
					state = 3;
					break;
				case 3: // inside object 2
					x2 += x; n2++;
					break;
				case 4: // past object 2
					std::cerr<<"More than two objects found!\n\r";
					break;
				default:
					std::cerr<<"Wrong scan state on green RGB("
					<<ColorIntensity[2]<<','
					<<ColorIntensity[1]<<','
					<<ColorIntensity[0]<<"): "<<state<<"\n\r";
				}	
			} else {
				switch (state) {
				// current pixel color is not the target color
				case 1: // inside obj 1
					state = 2;
					break;
				case 3: // inside obj 2
					state = 4;
					break;
				default:
					if (state > 4) {
						std::cerr<<"Wrong scan state on RGB("
					<<ColorIntensity[2]<<','
					<<ColorIntensity[1]<<','
					<<ColorIntensity[0]<<"): "<<state<<"\n\r";
					}
				}
			}
		}//endfor x
		//
		// Compute average x-locations
		// for the two objects
		//
		if (state == 4) {// both objects are visible
			// comute the average positions and number of hits
			if (n1 > 0) {
				x1a += (int) ((float)x1 / (float) n1);
				m1++; // number of hits over all y-scans
			}
			if (n2 > 0) {
				x2a += (int) ((float)x2 / (float) n2);
				m2++;
			}
		}
	}//endofr y

	if (m1 > 0 && m2 > 0) { // target acquired
		//
		// Navigate toward the target
		//
		x1a /= m1; // this completes the averaging
		x2a /= m2;
		int 
			target_position = 0.5*(x1a + x2a), // center of both ojbects
			target_size = fabs((double)x1a - (double)x2a),
			target_offset = desired_target_position - target_position,
			size_mismatch = desired_target_size - target_size;

		//
		// Check if the observed center is in alignment with the 
		// desired position in camera view
		//
		if (fabs(target_offset) > min_offset) {
			// turn toward the target:
			angle = sign(target_offset) * turning_angle;
			velocity = 0.0;
		} else {	// the target is straight on -
			// make a step toward the target
			angle = 0.0;
			if (fabs(size_mismatch) > min_size_mismatch) {
				velocity = sign(size_mismatch) * VISUAL_SPEED;
			} else { // we are at the target - stop here.
				std::cout << "\n\rTarget Reached!\n\r";std::cout.flush();
				reached_target = true;
				angle = 0.0;
				velocity = 0.0;
			}
		}
	} else {// target is not on the image: keep turning
		angle = turning_angle;
		velocity = 0.0;
	}
	delete img;

	camera_off = true;

}
} // end namespace cv

