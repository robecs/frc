
#define MODEL_NAME "robot"
#define CMD_TOPIC "velocity"
#define CAMERA_TOPIC "base/Camera/image"
#define IMAGE_FILE "img"

//
// PARAMETERS FOR AUTONOMOUS DRIVE
//
// Determine the time and speed to move without camera:
#define BLIND_TIME	2.5
#define BLIND_SPEED	4.0

// The same with the camera when spotting the target
// and moving toward it:
#define VISUAL_TIME	27.0
#define VISUAL_SPEED	2.0

//
// Target color and contrast
//

#define TARGET_COLOR	1
// 0 = red
// 1 = green
// 2 = blue

#define TARGET_COLOR_CONTRAST	0.5
// = target_color_intensity / target_light_intesity
// where target_light_intensity 
// = blue_intesity + green_intesity + red_intensity

// Next parameter is used to determine the distance
// to the target at which the robot has to stop.
// Since it is hard to estimate the distance to the target
// with a single-camera opencv, one can adjust the 
// desired target image size by try-and-arror
// The parameter below is in pixel size on the screen
#define DESIRED_TARGET_IMAGE_SIZE	75

