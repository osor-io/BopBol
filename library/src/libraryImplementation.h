#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include "utils.h"
#include "types.h"
#include "error.h"
#include "imageDetectionLibrary.h"
#include <mutex>

using namespace cv;
using namespace std;

#define DEQUE_SIZE_FOR_COLLISION 3
#define COLLISION_PAST_STEPS 1
#define NUM_FRAMES_SHOW_COLLISION 20
#define NUM_FRAMES_LOST_BALL 7
#define CALIBRATION_WARMUP 20


#define CIRCLE_CONTOUR_LIMIT 3
#define EPSILON_MULTIPLIER 6
#define EPSILON_DIV (float)1000.0f

#define RADIUS_LATERAL_MULT 0.66f



//
//	@@MAIN:
//	
//	So this program is meant to detect a ball colliding with a wall by detecting
//	sudden changes in its horizontal direction when it bounces. The ball should bounce, 
//	in other case there is no change in horizontal direction that can count as a collision.
//
//	In case we need to detect other types of movement, there is a change we are going to be able
//	to do it easily since we have a very accurate tracking of the ball, we should just need to adjust
//	the collision detection section
//
//
//	This program can take either a video argument with the color thresholds to track the vall in the video
//	or it can also take just the two thresholds, in the latter case it will try to use the webcam of the pc
//	to obtain the image. Tracking the ball with said color in real time.
//
//







//
//	============================================
//					LIBRARY STATE
//	============================================
//

struct ConfigurationParameters {

	//
	//	True if we want to show the red dot where the collision has 
	//	been detected.
	//
	int show_collisions = 1;

	//
	//	True if using pre recorded video and
	//	false if using the webcam.
	//
	bool using_video_file = false;

	//
	//	True to see if the trackbars for the configuration should be shown.
	//
	bool show_trackbars = true;

	//
	//	True if we want to show the processed frames in a separate.
	//	window.
	//
	bool output_frames = true;

	//
	//	The preferred internal resolution for the resizing of the
	//	frames when processing the images and calibrating.
	//
	int target_internal_resolution = 480;
};

struct CallbackFunctionPointers {
	COORDINATE_CALLBACK coordinate_callback = NULL;
	ERROR_CALLBACK error_callback = NULL;
};

struct CalibrationState {
	vector<vector<Point2f>> square_points;
	vector<Point2f> average_points;

	Mat homography_matrix;

	bool have_matrix = false;
};

struct ContourParameters {
	int circle_contour_limit = CIRCLE_CONTOUR_LIMIT;
	int epsilon_multiplier = EPSILON_MULTIPLIER;
};


struct LibraryState {

	BallDetectionParameters s_ball_detection_parameters;
	ConfigurationParameters s_configuration_parameters;
	CallbackFunctionPointers s_callback_functions;
	CalibrationState s_calibration_state;
	ContourParameters s_contour_parameters;

	//
	//	Mutex to protect parameters that can be changed by
	//	the client. More fine grained and efficient protection
	//	can be achieved with more that one mutex but in this case
	//	the configuration won't be done during image processing so
	//	caring about this is not necessary.
	//
	std::mutex s_configuration_mutex;



	//
	//	Stores the last N positions of the ball.
	//
	Deque s_main_deque;

	//
	//	The amount of frames that we have not seen the ball
	//
	int s_lost_ball_for_frames = 0;

	//
	//	Wether or not we had a ball in the previous frame
	//
	bool s_had_ball_previous_frame = false;

	//
	//	Stores the coordinates in screen space in which
	//	the last collision happened.
	//
	Point2f s_last_collision_coordinates = (-1.f, -1.f);

	//
	//	Stores the amount of frames remaining in which
	//	we have to show that a collision happened.
	//
	unsigned short s_frames_remaining_collision = 0;

	//
	//	Our variable that will contain the video, being that
	//	a premade one or a real time source.
	//
	VideoCapture *s_video = NULL;

	//
	//	When true, stops the video processing by ending the
	//	parse_frame loop.
	//
	bool s_should_stop = false;

	//
	//	When true it indicates that the image processing
	//	is still running.
	//
	bool s_running = false;


	//
	//	Calibration parameters
	//
	bool s_is_calibrating_projection = false;

};



STATE_TYPE createLibraryState();

void destroyLibraryState(STATE_TYPE library_state);

int initImageDetection(STATE_TYPE library_state);

int launchImageProcessing(STATE_TYPE library_state);

int stopImageDetection(STATE_TYPE library_state);

int setBall_HSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV);

int setBall_RadiusThreshold(
	STATE_TYPE library_state,
	int radius);

int setConfigurationParameters(
	STATE_TYPE library_state,
	int show_collisions,
	bool using_video_file,
	bool show_trackbars,
	bool output_frames);

int setCoordinateCallback(
	STATE_TYPE library_state,
	COORDINATE_CALLBACK callback_function_ptr);

int setErrorCallback(
	STATE_TYPE library_state,
	ERROR_CALLBACK callback_function_ptr);

int startProjectionCalibration(
	STATE_TYPE library_state);

ProjectionCalibration endProjectionCalibration(
	STATE_TYPE library_state);

int calibrateProjectionWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold);

int calibrateProjectionWithHSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV);

int calibrateBallWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold);

CalibrationSettings getCalibrationSettings(
	STATE_TYPE library_state);

int setCalibrationSettings(
	STATE_TYPE library_state,
	CalibrationSettings calibration_settings);







// ==========================
//		LOCAL FUNCTIONS
// ==========================


int parse_frame(STATE_TYPE library_state);

//
//	Prints the usage of this program in the command line
//
void showUsage();

//
//	Parses string with the format "h,s,v" to a Scalar with those HSV values
//
void parseHSVColor(char* string, Scalar * color);
