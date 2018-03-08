#include "bopbol.h"
#include "utils.h"
#include "types.h"
#include "error.h"
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <mutex>


//
//  ============================================
//           INTERNAL CONFIG PARAMETERS
//  ============================================
//

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
//  ============================================
//             INTERNAL STRUCTURES
//  ============================================
//

struct MouseClick {
	cv::Point2i point;
	bool clicked = false;
};

//
//  ============================================
//                LIBRARY STATE
//  ============================================
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
	BbCoordinateCallback coordinate_callback = NULL;
	BbErrorCallback error_callback = NULL;
};

struct CalibrationState {
	std::vector<std::vector<cv::Point2f>> square_points;
	std::vector<cv::Point2f> average_points;

	cv::Mat homography_matrix;

	bool have_matrix = false;
};

struct ContourParameters {
	int circle_contour_limit = CIRCLE_CONTOUR_LIMIT;
	int epsilon_multiplier = EPSILON_MULTIPLIER;
};

struct BbInstance_T {

	BbBallDetectionParameters s_ball_detection_parameters;
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
	cv::Point2f s_last_collision_coordinates = (-1.f, -1.f);

	//
	//	Stores the amount of frames remaining in which
	//	we have to show that a collision happened.
	//
	unsigned short s_frames_remaining_collision = 0;

	//
	//	Our variable that will contain the video, being that
	//	a premade one or a real time source.
	//
	cv::VideoCapture *s_video = NULL;

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


//
//  ============================================
//           DECLARING INTERNAL FUNCTIONS
//  ============================================
//

/**
Reads the position of the mouse and returns it in the struct

@param The even received from the click
@param mouse X coordinate of the click
@param mouse X coordinate of the click
@param flags set for the current event
@param The pointer to the struct MouseClick to fill
@see MouseClick
*/
void onMouse(int event, int x, int y, int flags, void* param);

/**
@Reorders the HSV ranges in case the lower and higher
values are switched i.e. lower is bigger than higher.

@param The instance of the library to use
*/
void reorderBallRanges(BbInstance_T* instance);

/**
Parses one frame of the image retrieved by the webcam.

@param The instance of the library to use
@return BbResult indicating the result of the function
*/
BbResult parseFrame(BbInstance_T* instance);

/**
Prints the usage of this program in the command line
*/
void showUsage();

/**
Parses string with the format "h,s,v" to a Scalar with those HSV values
*/
void parseHSVColor(char* string, cv::Scalar * color);


/**
Parses the instance handle to our real internal pointer type

@param The external instance handle
@return The internal instance type
*/
inline BbInstance_T* castInstance(BbInstance instance) {
	return reinterpret_cast<BbInstance_T*>(instance);
}

//
//  ============================================
//           DEFINING EXTERNAL FUNCTIONS
//  ============================================
//

bool bbIsCallable() {
	return true;
}

BbResult bbInit(BbInstance a_instance) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;


	instance->s_configuration_mutex.lock();


	//
	//	This is our circular queue that will store the
	//	last DEQUE_LENGTH positions of the ball in the frame 
	//
	deque_init(&instance->s_main_deque);

	//
	//	Simply opening the video source for the webcam since
	//	we are always calling from Unity
	//
	instance->s_video->open(0);



	//
	// We check if we can read the video
	//
	if (!instance->s_video->isOpened()) {
		std::cout << "Could not read video file" << std::endl;
		manageError(instance->s_callback_functions.error_callback, BbError::UNABLE_TO_OPEN_VIDEO);

		instance->s_configuration_mutex.unlock();
		return BB_FAILURE;
	}

	instance->s_video->release();


	instance->s_configuration_mutex.unlock();


	return BB_SUCCESS;

}

BbResult bbLaunch(BbInstance a_instance) {
	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	//
	//	Inside this function we are opening and releasing
	//	the video source to avoid accessing the webcam constantly.
	//
	instance->s_video->open(0);

	instance->s_should_stop = false;
	instance->s_running = true;

	//
	//	We destroy the previous windows that might me mangling
	//	arround.
	//
	cv::destroyAllWindows();

	//
	//	And we show the trackbars to give the option to change
	//	the parameters to detect the ball.
	//
	if (instance->s_configuration_parameters.show_trackbars) {

		//
		//	We give the option to change the hsv ranges with
		//	trackbars
		//
		cv::namedWindow("Control", CV_WINDOW_FREERATIO);
		cvCreateTrackbar("LowH", "Control", &instance->s_ball_detection_parameters.h_low, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control", &instance->s_ball_detection_parameters.h_high, 179);
		cvCreateTrackbar("LowS", "Control", &instance->s_ball_detection_parameters.s_low, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control", &instance->s_ball_detection_parameters.s_high, 255);
		cvCreateTrackbar("LowV", "Control", &instance->s_ball_detection_parameters.v_low, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control", &instance->s_ball_detection_parameters.v_high, 255);
		cvCreateTrackbar("Radius", "Control", &instance->s_ball_detection_parameters.radius_threshold, 100);
		cvCreateTrackbar("Contour Limit", "Control", &instance->s_contour_parameters.circle_contour_limit, 20);
		cvCreateTrackbar("Epsilon Multiplier", "Control", &instance->s_contour_parameters.epsilon_multiplier, 200);
		cvCreateTrackbar("Collision", "Control", &instance->s_configuration_parameters.show_collisions, 1);
	}

	while (!instance->s_should_stop) {

		//
		//	For the time being we will just lock the configuration for a whole frame
		//	since we will not be changing it during processing very ofter (or ever).
		//
		instance->s_configuration_mutex.lock();
		parseFrame(instance);
		instance->s_configuration_mutex.unlock();

	}

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	cv::destroyAllWindows();

	instance->s_video->release();

	instance->s_running = false;

	return BB_SUCCESS;
}

BbResult bbStop(BbInstance a_instance) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	instance->s_should_stop = true;

	while (instance->s_running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	cv::destroyAllWindows();

	return BB_SUCCESS;
}

BbInstance bbCreateInstance() {

	BbInstance_T * instance = new BbInstance_T();

	if (instance == nullptr) {
		return nullptr;
	}

	instance->s_video = new cv::VideoCapture();

	return instance;
}

void bbDestroyInstance(BbInstance a_instance) {
	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return;

	delete instance->s_video;
	delete instance;
}

BbResult bbSetBallHSVRanges(
	BbInstance a_instance,
	int a_low_h, int a_high_h, int a_low_s,
	int a_high_s, int a_low_v, int a_high_v) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;


	instance->s_configuration_mutex.lock();

	instance->s_ball_detection_parameters.h_low = a_low_h;
	instance->s_ball_detection_parameters.h_high = a_high_h;
	instance->s_ball_detection_parameters.s_low = a_low_s;
	instance->s_ball_detection_parameters.s_high = a_high_s;
	instance->s_ball_detection_parameters.v_low = a_low_v;
	instance->s_ball_detection_parameters.v_high = a_high_v;

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbSetBallRadiusThreshold(
	BbInstance a_instance,
	int radius) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;


	instance->s_configuration_mutex.lock();

	instance->s_ball_detection_parameters.radius_threshold = radius;

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbSetConfigurationParameters(
	BbInstance a_instance,
	int show_collisions,
	bool using_video_file,
	bool show_trackbars,
	bool output_frames) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	instance->s_configuration_mutex.lock();

	instance->s_configuration_parameters.show_collisions = show_collisions;
	instance->s_configuration_parameters.using_video_file = using_video_file;
	instance->s_configuration_parameters.show_trackbars = show_trackbars;
	instance->s_configuration_parameters.output_frames = output_frames;

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbSetCoordinateCallback(
	BbInstance a_instance,
	BbCoordinateCallback callback_function_ptr) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	instance->s_configuration_mutex.lock();

	instance->s_callback_functions.coordinate_callback = callback_function_ptr;

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbSetErrorCallback(
	BbInstance a_instance,
	BbErrorCallback callback_function_ptr) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	instance->s_configuration_mutex.lock();

	instance->s_callback_functions.error_callback = callback_function_ptr;

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbStartAreaCalibration(
	BbInstance a_instance) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	instance->s_configuration_mutex.lock();

	//
	//	Inside this function we are opening and releasing
	//	the video source to avoid accessing the webcam constantly.
	//
	instance->s_video->open(0);

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	cv::destroyAllWindows();

	instance->s_is_calibrating_projection = true;

	instance->s_calibration_state.have_matrix = false;

	instance->s_calibration_state.square_points.clear();

	instance->s_calibration_state.average_points.clear();

	instance->s_calibration_state.homography_matrix = cv::Mat();

	//
	//	@TODO: Complete and probably erase the previous values
	//	that represent the calibration (Homography matrices, 
	//	points that define the squares, etc.).
	//

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbAreaCalibration bbEndAreaCalibration(
	BbInstance a_instance) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BbAreaCalibration{};


	std::vector<cv::Point2f> normal_values;
	normal_values.push_back(cv::Point2f(0.f, 1.f));
	normal_values.push_back(cv::Point2f(1.f, 1.f));
	normal_values.push_back(cv::Point2f(1.f, 0.f));
	normal_values.push_back(cv::Point2f(0.f, 0.f));


	instance->s_configuration_mutex.lock();


	if (instance->s_calibration_state.square_points.size() <= 0) {

		manageError(instance->s_callback_functions.error_callback, BbError::COULD_NOT_CALIBRATE);

		cv::destroyAllWindows();

		instance->s_video->release();

		instance->s_is_calibrating_projection = false;

		instance->s_calibration_state.have_matrix = true;
		instance->s_calibration_state.average_points.clear();
		instance->s_calibration_state.square_points.clear();
		instance->s_calibration_state.homography_matrix = cv::Mat();

		instance->s_configuration_mutex.unlock();

		return BbAreaCalibration();

	}

	instance->s_calibration_state.average_points.clear();
	instance->s_calibration_state.average_points.push_back(cv::Point2f(0.f, 0.f));
	instance->s_calibration_state.average_points.push_back(cv::Point2f(0.f, 0.f));
	instance->s_calibration_state.average_points.push_back(cv::Point2f(0.f, 0.f));
	instance->s_calibration_state.average_points.push_back(cv::Point2f(0.f, 0.f));


	for (std::vector<cv::Point2f> vec : instance->s_calibration_state.square_points) {

		for (short i = 0; i < 4; i++) {
			instance->s_calibration_state.average_points[i] += vec[i];
		}

	}

	for (short i = 0; i < 4; i++) {
		instance->s_calibration_state.average_points[i] /= (float)instance->s_calibration_state.square_points.size();
	}

	utilscv_sortSquarePoints(&(instance->s_calibration_state.average_points));

	BbAreaCalibration projection_calibration;

	projection_calibration.point_0.x = instance->s_calibration_state.average_points[0].x;
	projection_calibration.point_0.y = instance->s_calibration_state.average_points[0].y;

	projection_calibration.point_1.x = instance->s_calibration_state.average_points[1].x;
	projection_calibration.point_1.y = instance->s_calibration_state.average_points[1].y;

	projection_calibration.point_2.x = instance->s_calibration_state.average_points[2].x;
	projection_calibration.point_2.y = instance->s_calibration_state.average_points[2].y;

	projection_calibration.point_3.x = instance->s_calibration_state.average_points[3].x;
	projection_calibration.point_3.y = instance->s_calibration_state.average_points[3].y;


	instance->s_calibration_state.homography_matrix = findHomography(instance->s_calibration_state.average_points, normal_values);

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	cv::destroyAllWindows();

	instance->s_video->release();

	instance->s_is_calibrating_projection = false;

	instance->s_calibration_state.have_matrix = true;

	instance->s_configuration_mutex.unlock();

	return projection_calibration;
}

BbResult bbCalibrateAreaWithClick(
	BbInstance a_instance,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

		cv::Mat clean_frame, frame, mask;

		instance->s_configuration_mutex.lock();

		if (!instance->s_is_calibrating_projection) {
			manageError(instance->s_callback_functions.error_callback, BbError::NOT_IN_CALIBRATION_MODE);
			instance->s_configuration_mutex.unlock();
			return BB_FAILURE;
		}


		//
		//	We wait a bit to have a clear frame
		//
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		for (short i = 0; i < CALIBRATION_WARMUP; i++) {
			if (!instance->s_video->read(clean_frame)) {
				manageError(instance->s_callback_functions.error_callback, BbError::COULD_NOT_READ_FRAME);
				return BB_FAILURE;
			}
			cv::waitKey(1);
		}


		cv::Vec3b hsv_base;
		MouseClick click;
		while (instance->s_video->read(clean_frame)) {
			imshow("Click on the projection", clean_frame);
			cv::setMouseCallback("Click on the projection", onMouse, &click);
			if (click.clicked) {
				cv::Vec3b rgb = clean_frame.at<cv::Vec3b>((click.point));
				cv::Mat HSV;
				cv::Mat RGB = clean_frame(cv::Rect(click.point.x, click.point.y, 1, 1));
				cvtColor(RGB, HSV, CV_BGR2HSV);
				hsv_base = HSV.at<cv::Vec3b>(0, 0);
				break;
			}
			int k = cv::waitKey(10);
		}

		//
		//	We resize the frame to avoid tough computations
		//	and we still have to decide if we need to blur it
		//	the code is
		//		blur(frame, frame, Size(2, 2));
		//	But we won't do it for now
		//
		//	@@TODO: Check if this gives us better or worse results
		//	Or more consistent ones for that matter
		//
		utilscv_resize(&clean_frame, instance->s_configuration_parameters.target_internal_resolution);


		//
		//	Change the frame to HSV color format
		//
		cvtColor(clean_frame, frame, CV_BGR2HSV);


		//
		//	And we get our frame with only the ball's pixels, we do some
		//	erosions and dilations to avout bumps and stuff
		//
		cv::inRange(frame,
			cv::Scalar(hsv_base[0] - hue_threshold,
				hsv_base[1] - saturation_threshold,
				hsv_base[2] - value_threshold),
			cv::Scalar(hsv_base[0] + hue_threshold,
				hsv_base[1] + saturation_threshold,
				hsv_base[2] + value_threshold),
			mask);



		//
		//	@@OPTIMIZATION: CHeck if this gives us better or worse results
		//	so far I think it does not so probably just leave it out :D
		//
		//			erode(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);
		//			dilate(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);


		//
		//	@@SCOPE
		//	Here we find the properties that represent the 4-sided rectangle
		//
		{
			//
			//	Output Vectors for the contours
			//
			std::vector<std::vector<cv::Point>> contours;
			std::vector<cv::Point2f> processed_contour;
			bool found_valid_contour = false;
			std::vector<cv::Vec4i> hierarchy;

			//
			//	We look for the external contours
			//
			findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			if (contours.size() > 0) {

				//
				//	We take the biggest rectangle
				//
				float largest_area = 0.0f;
				int largest_contour_index;
				for (int i = 0; i < contours.size(); i++) {

					std::vector<cv::Point> aux_processed_contour;

					//
					//	Now we use approxPoly to get the processed rectangle
					//	
					approxPolyDP(cv::Mat(contours[i]), aux_processed_contour, 3, true);

					//
					//	Now if we have a 4 sided polygon
					//
					if (aux_processed_contour.size() == 4) {

						found_valid_contour = true;

						//
						//	We pick the largest one
						//
						float a = (float)contourArea(aux_processed_contour, false);
						if (a > largest_area) {
							largest_area = a;
							largest_contour_index = i;
							processed_contour.clear();
							for (short i = 0; i < 4; i++) {
								processed_contour.push_back(aux_processed_contour[i]);
							}
						}
					}
				}

				//
				//	Now we should have the points on processed_contour
				//
				if (found_valid_contour && processed_contour.size() == 4) {

					utilscv_sortSquarePoints(&processed_contour);

					if (instance->s_configuration_parameters.output_frames) {
						cv::circle(clean_frame, processed_contour[0], 3, cv::Scalar(255, 0, 0), -1);
						cv::circle(clean_frame, processed_contour[1], 3, cv::Scalar(0, 255, 0), -1);
						cv::circle(clean_frame, processed_contour[2], 3, cv::Scalar(255, 0, 255), -1);
						cv::circle(clean_frame, processed_contour[3], 3, cv::Scalar(0, 0, 255), -1);
					}

					//
					//	We sabe the calibration points for this iteration of the calibration
					//
					instance->s_calibration_state.square_points.push_back(processed_contour);
				}


			}

			//
			//	Finally we can show the frames if we want
			//
			if (instance->s_configuration_parameters.output_frames) {
				imshow("clean_frame", clean_frame);
				imshow("mask", mask);
			}

			cv::waitKey(1);

		}

		//
		//	ANd we wait a bit after the calibration to be able to see
		//	the frames if needed
		//
		if (instance->s_configuration_parameters.output_frames) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}

		instance->s_configuration_mutex.unlock();

		return BB_SUCCESS;
	
}

BbResult bbCalibrateAreaWithHSVRanges(
	BbInstance a_instance,
	int iLowH, int a_high_h, int a_low_s,
	int a_high_s, int a_low_v, int a_high_v) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	cv::Mat clean_frame, frame, mask;


	instance->s_configuration_mutex.lock();

	if (!instance->s_is_calibrating_projection) {
		manageError(instance->s_callback_functions.error_callback, BbError::NOT_IN_CALIBRATION_MODE);
		instance->s_configuration_mutex.unlock();
		return BB_FAILURE;
	}


	//
	//	We wait a bit to have a clear frame
	//
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (short i = 0; i < CALIBRATION_WARMUP; i++) {
		if (!instance->s_video->read(clean_frame)) {
			manageError(instance->s_callback_functions.error_callback, BbError::COULD_NOT_READ_FRAME);
			return BB_FAILURE;
		}
		cv::waitKey(1);
	}


	//
	//	We resize the frame to avoid tough computations
	//	and we still have to decide if we need to blur it
	//	the code is
	//		blur(frame, frame, Size(2, 2));
	//	But we won't do it for now
	//
	//	@@TODO: Check if this gives us better or worse results
	//	Or more consistent ones for that matter
	//
	utilscv_resize(&clean_frame, instance->s_configuration_parameters.target_internal_resolution);


	//
	//	Change the frame to HSV color format
	//
	cv::cvtColor(clean_frame, frame, CV_BGR2HSV);


	//
	//	And we get our frame with only the ball's pixels, we do some
	//	erosions and dilations to avout bumps and stuff
	//
	cv::inRange(frame,
		cv::Scalar(iLowH,
			a_low_s,
			a_low_v),
		cv::Scalar(a_high_h,
			a_high_s,
			a_high_v),
		mask);



	//
	//	@@OPTIMIZATION: CHeck if this gives us better or worse results
	//	so far I think it does not so probably just leave it out :D
	//
	//			erode(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);
	//			dilate(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);


	//
	//	@@SCOPE
	//	Here we find the properties that represent the 4-sided rectangle
	//
	{
		//
		//	Output Vectors for the contours
		//
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Point2f> processed_contour;
		bool found_valid_contour = false;
		std::vector<cv::Vec4i> hierarchy;

		//
		//	We look for the external contours
		//
		cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		if (contours.size() > 0) {

			//
			//	We take the biggest rectangle
			//
			float largest_area = 0.0f;
			int largest_contour_index;
			for (int i = 0; i < contours.size(); i++) {

				std::vector<cv::Point> aux_processed_contour;

				//
				//	Now we use approxPoly to get the processed rectangle
				//	
				cv::approxPolyDP(cv::Mat(contours[i]), aux_processed_contour, 3, true);

				//
				//	Now if we have a 4 sided polygon
				//
				if (aux_processed_contour.size() == 4) {

					found_valid_contour = true;

					//
					//	We pick the largest one
					//
					float a = static_cast<float>(cv::contourArea(aux_processed_contour, false));
					if (a > largest_area) {
						largest_area = a;
						largest_contour_index = i;
						processed_contour.clear();
						for (short i = 0; i < 4; i++) {
							processed_contour.push_back(aux_processed_contour[i]);
						}
					}
				}
			}

			//
			//	Now we should have the points on processed_contour
			//
			if (found_valid_contour && processed_contour.size() == 4) {

				utilscv_sortSquarePoints(&processed_contour);

				if (instance->s_configuration_parameters.output_frames) {
					cv::circle(clean_frame, processed_contour[0], 3, cv::Scalar(255, 0, 0), -1);
					cv::circle(clean_frame, processed_contour[1], 3, cv::Scalar(0, 255, 0), -1);
					cv::circle(clean_frame, processed_contour[2], 3, cv::Scalar(255, 0, 255), -1);
					cv::circle(clean_frame, processed_contour[3], 3, cv::Scalar(0, 0, 255), -1);
				}

				//
				//	We sabe the calibration points for this iteration of the calibration
				//
				instance->s_calibration_state.square_points.push_back(processed_contour);
			}


		}

		//
		//	Finally we can show the frames if we want
		//
		if (instance->s_configuration_parameters.output_frames) {
			cv::imshow("clean_frame", clean_frame);
			cv::imshow("mask", mask);
		}

		cv::waitKey(1);

	}

	//
	//	ANd we wait a bit after the calibration to be able to see
	//	the frames if needed
	//
	if (instance->s_configuration_parameters.output_frames) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	else {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

	}

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbResult bbCalibrateBallWithClick(
	BbInstance a_instance,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;
	
	cv::Mat clean_frame, frame, mask;

	instance->s_configuration_mutex.lock();
	{

		instance->s_video->open(0);

		//
		//	We wait a bit to have a clear frame
		//
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		for (short i = 0; i < CALIBRATION_WARMUP; i++) {
			if (!instance->s_video->read(clean_frame)) {
				manageError(instance->s_callback_functions.error_callback, BbError::COULD_NOT_READ_FRAME);
				return BB_FAILURE;
			}
			cv::waitKey(1);
		}


		cv::Vec3b hsv_base_low;
		MouseClick click_low;
		while (instance->s_video->read(clean_frame)) {
			imshow("Click on dark version of the ball", clean_frame);
			cv::setMouseCallback("Click on dark version of the ball", onMouse, &click_low);
			if (click_low.clicked) {
				cv::Vec3b rgb = clean_frame.at<cv::Vec3b>((click_low.point));
				cv::Mat HSV;
				cv::Mat RGB = clean_frame(cv::Rect(click_low.point.x, click_low.point.y, 1, 1));
				cv::cvtColor(RGB, HSV, CV_BGR2HSV);
				hsv_base_low = HSV.at<cv::Vec3b>(0, 0);
				break;
			}
			int k = cv::waitKey(10);
		}

		cv::Vec3b hsv_base_high;
		MouseClick click_high;
		while (instance->s_video->read(clean_frame)) {
			imshow("Click on lit version of the ball", clean_frame);
			cv::setMouseCallback("Click on lit version of the ball", onMouse, &click_high);
			if (click_high.clicked) {
				cv::Vec3b rgb = clean_frame.at<cv::Vec3b>((click_high.point));
				cv::Mat HSV;
				cv::Mat RGB = clean_frame(cv::Rect(click_high.point.x, click_high.point.y, 1, 1));
				cv::cvtColor(RGB, HSV, CV_BGR2HSV);
				hsv_base_high = HSV.at<cv::Vec3b>(0, 0);
				break;
			}
			int k = cv::waitKey(10);
		}

		instance->s_ball_detection_parameters.h_low = hsv_base_low[0];
		instance->s_ball_detection_parameters.s_low = hsv_base_low[1];
		instance->s_ball_detection_parameters.v_low = hsv_base_low[2];

		instance->s_ball_detection_parameters.h_high = hsv_base_high[0];
		instance->s_ball_detection_parameters.s_high = hsv_base_high[1];
		instance->s_ball_detection_parameters.v_high = hsv_base_high[2];

		reorderBallRanges(instance);

		instance->s_ball_detection_parameters.h_low -= hue_threshold;
		instance->s_ball_detection_parameters.s_low -= saturation_threshold;
		instance->s_ball_detection_parameters.v_low -= value_threshold;

		instance->s_ball_detection_parameters.h_high += hue_threshold;
		instance->s_ball_detection_parameters.s_high += saturation_threshold;
		instance->s_ball_detection_parameters.v_high += value_threshold;

		//
		//	@@DOING: Setting saturation and value to broad ranges
		//
		instance->s_ball_detection_parameters.s_low = 100;
		instance->s_ball_detection_parameters.s_high = 255;
		instance->s_ball_detection_parameters.v_low = 30;
		instance->s_ball_detection_parameters.v_high = 255;


		instance->s_video->release();

		cv::destroyAllWindows();
	}

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}

BbCalibrationSettings bbGetCalibrationSettings(BbInstance a_instance) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BbCalibrationSettings{};

	BbCalibrationSettings calibration_settings;

	instance->s_configuration_mutex.lock();

	{

		BbAreaCalibration projection_calibration_aux;

		if (instance->s_calibration_state.have_matrix && instance->s_calibration_state.average_points.size() > 0) {
			//
			//	Setting the points that define the square on the screen
			//
			projection_calibration_aux.point_0.x = instance->s_calibration_state.average_points[0].x;
			projection_calibration_aux.point_0.y = instance->s_calibration_state.average_points[0].y;

			projection_calibration_aux.point_1.x = instance->s_calibration_state.average_points[1].x;
			projection_calibration_aux.point_1.y = instance->s_calibration_state.average_points[1].y;

			projection_calibration_aux.point_2.x = instance->s_calibration_state.average_points[2].x;
			projection_calibration_aux.point_2.y = instance->s_calibration_state.average_points[2].y;

			projection_calibration_aux.point_3.x = instance->s_calibration_state.average_points[3].x;
			projection_calibration_aux.point_3.y = instance->s_calibration_state.average_points[3].y;

			projection_calibration_aux.valid = true;

			calibration_settings.projection_calibration = projection_calibration_aux;
		}
		else {
			projection_calibration_aux.valid = false;
		}

		//
		//	Setting the ball parameters
		//
		calibration_settings.ball_detection_parameters = instance->s_ball_detection_parameters;
	}

	instance->s_configuration_mutex.unlock();

	return calibration_settings;
}

BbResult bbSetCalibrationSettings(BbInstance a_instance, BbCalibrationSettings calibration_settings) {

	BbInstance_T* instance = castInstance(a_instance);
	if (instance == nullptr) return BB_FAILURE;

	std::vector<cv::Point2f> normal_values;
	normal_values.push_back(cv::Point2f(0.f, 1.f));
	normal_values.push_back(cv::Point2f(1.f, 1.f));
	normal_values.push_back(cv::Point2f(1.f, 0.f));
	normal_values.push_back(cv::Point2f(0.f, 0.f));

	instance->s_configuration_mutex.lock();

	{

		instance->s_ball_detection_parameters = calibration_settings.ball_detection_parameters;

		if (calibration_settings.projection_calibration.valid) {

			BbAreaCalibration projection_calibration_aux = calibration_settings.projection_calibration;

			instance->s_calibration_state.square_points.clear();

			instance->s_calibration_state.average_points.clear();
			instance->s_calibration_state.average_points.resize(4);

			instance->s_calibration_state.average_points[0].x = (float)projection_calibration_aux.point_0.x;
			instance->s_calibration_state.average_points[0].y = (float)projection_calibration_aux.point_0.y;

			instance->s_calibration_state.average_points[1].x = (float)projection_calibration_aux.point_1.x;
			instance->s_calibration_state.average_points[1].y = (float)projection_calibration_aux.point_1.y;

			instance->s_calibration_state.average_points[2].x = (float)projection_calibration_aux.point_2.x;
			instance->s_calibration_state.average_points[2].y = (float)projection_calibration_aux.point_2.y;

			instance->s_calibration_state.average_points[3].x = (float)projection_calibration_aux.point_3.x;
			instance->s_calibration_state.average_points[3].y = (float)projection_calibration_aux.point_3.y;

			instance->s_calibration_state.homography_matrix = findHomography(instance->s_calibration_state.average_points, normal_values);

			instance->s_calibration_state.have_matrix = true;
		}
		else {
			instance->s_calibration_state.have_matrix = false;
		}
	}

	instance->s_configuration_mutex.unlock();

	return BB_SUCCESS;
}


//
//  ============================================
//           DEFINING INTERNAL FUNCTIONS
//  ============================================
//

void onMouse(int event, int x, int y, int flags, void* param) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		MouseClick* data = (MouseClick*)(param);
		data->point.x = x;
		data->point.y = y;
		data->clicked = true;
	}
}

void reorderBallRanges(BbInstance_T* state) {

	if (state->s_ball_detection_parameters.h_low > state->s_ball_detection_parameters.h_high) {
		int aux = state->s_ball_detection_parameters.h_low;
		state->s_ball_detection_parameters.h_low = state->s_ball_detection_parameters.h_high;
		state->s_ball_detection_parameters.h_high = aux;
	}

	if (state->s_ball_detection_parameters.s_low > state->s_ball_detection_parameters.s_high) {
		int aux = state->s_ball_detection_parameters.s_low;
		state->s_ball_detection_parameters.s_low = state->s_ball_detection_parameters.s_high;
		state->s_ball_detection_parameters.s_high = aux;
	}

	if (state->s_ball_detection_parameters.v_low > state->s_ball_detection_parameters.v_high) {
		int aux = state->s_ball_detection_parameters.v_low;
		state->s_ball_detection_parameters.v_low = state->s_ball_detection_parameters.v_high;
		state->s_ball_detection_parameters.v_high = aux;
	}

}

BbResult parseFrame(BbInstance_T* instance) {

	cv::Mat clean_frame, frame, mask;


	if (!instance->s_video->read(clean_frame)) {
		manageError(instance->s_callback_functions.error_callback, BbError::COULD_NOT_READ_FRAME);
		return BB_FAILURE;
	}


	//
	//	We resize the frame to avoid tough computations
	//	and we still have to decide if we need to blur it
	//	the code is
	//		blur(frame, frame, Size(2, 2));
	//	But we won't do it for now
	//
	//	for resizing we either use
	utilscv_resize(&clean_frame, instance->s_configuration_parameters.target_internal_resolution);
	//	or
	//utilscv_resizeCloseTo(&clean_frame, state->s_configuration_parameters.target_internal_resolution);


	//
	//	Change the frame to HSV color format
	//
	cvtColor(clean_frame, frame, CV_BGR2HSV);


	//
	//	And we get our frame with only the ball's pixels, we do some
	//	erosions and dilations to avout bumps and stuff
	//
	cv::inRange(frame,
		cv::Scalar(instance->s_ball_detection_parameters.h_low,
			instance->s_ball_detection_parameters.s_low,
			instance->s_ball_detection_parameters.v_low),
		cv::Scalar(instance->s_ball_detection_parameters.h_high,
			instance->s_ball_detection_parameters.s_high,
			instance->s_ball_detection_parameters.v_high),
		mask);

	cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);


	//
	//	We just care about the radius and the centre
	//	so we can open a new scope
	//
	cv::Point2f center, centroid(-1.f, -1.f);
	float radius = 0.0f;
	bool found_circle = false;

	//
	//	@@SCOPE
	//	Here we find the properties of the circle that encloses the ball
	//
	{

#ifdef HOUGH

		std::vector<cv::Vec3f> circles;

		cv::Mat hough_frame = mask;


		cv::erode(hough_frame, hough_frame, cv::Mat(), cv::Point(-1, -1), 6, 1, 1);
		cv::dilate(hough_frame, hough_frame, cv::Mat(), cv::Point(-1, -1), 6, 1, 1);
		cv::GaussianBlur(hough_frame, hough_frame, cv::Size(9, 9), 2, 2);


		imshow("test blurred", hough_frame);
		cv::waitKey(10);

		cv::HoughCircles(hough_frame, circles, CV_HOUGH_GRADIENT, 1, mask.rows / 8, 100, 20, 0, 0);

		for (size_t i = 0; i < circles.size(); i++)
		{
			//float radius_aux = cvRound(circles[i][2]);

			float radius_aux = circles[i][2];

			//
			//	We could probably remove this check since cv::HoughCircles is likely already doing it.
			//
			if (radius_aux > instance->s_ball_detection_parameters.radius_threshold) {
				cv::Point center_aux(cvRound(circles[i][0]), cvRound(circles[i][1]));
				radius = radius_aux;
				centroid = center_aux;
				found_circle = true;
			}

		}

#else	//We default to the contours method


		//
		//	Output Vectors for the contours
		//
		std::vector< std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		//
		//	We look for the external contours
		//
		cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		std::vector<std::vector<cv::Point>> approx_contours;

		if (contours.size() > 0) {

			approx_contours.resize(contours.size());

			//
			//	And we take the biggest one, here we can decide to detect as
			//	many balls as needed calculating the N biggest contours
			//
			float largest_area = 0.0f;
			int largest_contour_index = -1;
			for (int i = 0; i < contours.size(); i++) {
				float epsilon = (float)instance->s_contour_parameters.epsilon_multiplier / EPSILON_DIV * (float)arcLength(contours[i], true);
				approxPolyDP(contours[i], approx_contours[i], epsilon, true);
				float area = (float)contourArea(contours[i], false);

				if (area > largest_area && approx_contours[i].size() > instance->s_contour_parameters.circle_contour_limit) {
					largest_area = area;
					largest_contour_index = i;
				}

			}

			if (instance->s_configuration_parameters.output_frames) {
				drawContours(clean_frame, approx_contours, -1, cv::Scalar(0, 255, 0));
			}


			if (largest_contour_index >= 0) {
				//
				//	And we take the minimum enclosing circle for our contour
				//	to be able to calculate its centroid
				//
				cv::minEnclosingCircle(contours[largest_contour_index], center, radius);
				cv::Moments moments = cv::moments(contours[largest_contour_index]);
				centroid = cv::Point2f((float)moments.m10 / (float)moments.m00, (float)moments.m01 / (float)moments.m00);

				if (radius > instance->s_ball_detection_parameters.radius_threshold) {
					found_circle = true;
				}
			}

		}

#endif

	}


	//
	//	@@SCOPE
	//	Here we calculate the collisions and draw everything accordingly
	//
	{
		//
		//	The future arguments for printing the markers on the ball
		//
		cv::Scalar circle_color(255, 255, 0);
		cv::Scalar centroid_color(255, 0, 0);
		int circle_thickness = 2;

		//
		//	We detect here if there has been a collision
		//
		if (found_circle && instance->s_main_deque.size > DEQUE_SIZE_FOR_COLLISION && centroid.x > 0.0f) {
			//
			//	For the old direction we can calculate the average of the last N frames
			//
			float old_direction = deque_getElementAt(&instance->s_main_deque, 1).x - deque_getElementAt(&instance->s_main_deque, 1 + COLLISION_PAST_STEPS).x;
			float curr_direction = centroid.x - deque_getElementAt(&instance->s_main_deque, 1).x;


			//
			//	We have a collision if
			//
			if (old_direction * curr_direction < 0.0f) {

				//
				//	So we update the collision coordinates and set up
				//	the amount of frames we want to show the collision for
				//
				instance->s_last_collision_coordinates = deque_getElementAt(&instance->s_main_deque, 1);

				//
				//	Correct for the depth of the ball
				//
				if (old_direction > 0) {
					instance->s_last_collision_coordinates.x += radius * RADIUS_LATERAL_MULT;
				}
				else {
					instance->s_last_collision_coordinates.x -= radius * RADIUS_LATERAL_MULT;
				}

				instance->s_frames_remaining_collision = NUM_FRAMES_SHOW_COLLISION;


				if (instance->s_calibration_state.have_matrix) {
					//
					//	Use OpenCV's findHomography and perspectiveTransform with
					//	the previously obtained Homography matrix to map screen coordinates
					//	to wall projection coordinates.
					//	
					std::vector<cv::Point2f> input_not_transformed;
					input_not_transformed.push_back(instance->s_last_collision_coordinates);
					std::vector<cv::Point2f> output_transformed;
					output_transformed.push_back(cv::Point2f(0, 0));
					perspectiveTransform(input_not_transformed, output_transformed, instance->s_calibration_state.homography_matrix);

					//
					//	And we use the CALLBACK if we have to
					//
					if (instance->s_callback_functions.coordinate_callback != NULL && !instance->s_should_stop) {

						auto collision_transformed = output_transformed[0];

						instance->s_callback_functions.coordinate_callback(
							collision_transformed.x,
							collision_transformed.y
						);
					}
				}

			}
		}


		//
		//	With the data from the circle and the centroid we can draw it
		//
		if (found_circle && radius > instance->s_ball_detection_parameters.radius_threshold) {

			if (instance->s_configuration_parameters.show_collisions) {
				cv::circle(clean_frame, center, (int)radius, circle_color, circle_thickness);
				cv::circle(clean_frame, centroid, 3, centroid_color, -1);
			}

			//
			//	We insert the element in the deque
			//
			deque_insertElement(&instance->s_main_deque, centroid);
			instance->s_had_ball_previous_frame = true;
		}
		else {

			//
			//	@@DOING: Give a bit more leniance to detect the ball if
			//	we loose it for a few frames.
			//

			if (instance->s_had_ball_previous_frame == true) {
				instance->s_lost_ball_for_frames = NUM_FRAMES_LOST_BALL;
			}
			else if (instance->s_lost_ball_for_frames > 0) {
				--instance->s_lost_ball_for_frames;
			}
			else {
				instance->s_lost_ball_for_frames = -1;
				deque_init(&instance->s_main_deque);
			}

			instance->s_had_ball_previous_frame = false;


		}
	}

	//
	//	And we print every line
	//
	if (instance->s_main_deque.size > 0)
		for (unsigned int i = 0; i < instance->s_main_deque.size - 1; i++) {
			cv::line(
				clean_frame,
				deque_getElementAt(&instance->s_main_deque, i),
				deque_getElementAt(&instance->s_main_deque, i + 1),
				cv::Scalar(255, 0, 255));
		}

	//
	//	We also print here the last collision coordinates
	//	if we still have remaining frames for that
	//
	if (instance->s_frames_remaining_collision > 0 && instance->s_configuration_parameters.show_collisions) {
		instance->s_frames_remaining_collision--;
		cv::circle(clean_frame, instance->s_last_collision_coordinates, 10, cv::Scalar(0, 0, 255), -1);
	}

	//
	//	We add the lines that define the projection rectangle
	//	with the perspective
	//
	if (instance->s_configuration_parameters.show_collisions
		&& instance->s_calibration_state.have_matrix
		&& instance->s_calibration_state.average_points.size() > 0) {
		cv::line(clean_frame,
			instance->s_calibration_state.average_points[0],
			instance->s_calibration_state.average_points[1], cv::Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			instance->s_calibration_state.average_points[1],
			instance->s_calibration_state.average_points[2], cv::Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			instance->s_calibration_state.average_points[2],
			instance->s_calibration_state.average_points[3], cv::Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			instance->s_calibration_state.average_points[3],
			instance->s_calibration_state.average_points[0], cv::Scalar(255, 100, 0), 4);
	}

	//
	//	Finally we can show the frame
	//
	if (instance->s_configuration_parameters.output_frames) {
		imshow("frame", clean_frame);
	}

	cv::waitKey(1);

	return BB_SUCCESS;

}

void showUsage() {
	std::cout << "\nThis program needs a source (the path) for the video, (or none for webcam)" << std::endl;
	std::cout << "EXAMPLES OF USAGE:"
		"\n\t With Video: $ executable.exe ./resources/video.mp4"
		"\n\t Without Video: $ executable.exe"
		<< std::endl;
	PRESS_TO_CONTINUE
}

void parseHSVColor(char* string, cv::Scalar * color) {

	//
	//	what an ugly function to just parse 3 numbers
	//	I know, but it's just used twice so let's optimize
	//	out time.
	//

	std::string cppstring{ string };
	std::string delimiter = ",";
	int array[3];

	size_t pos = 0;
	std::string token;
	for (short i = 0; i < 2; i++) {

		if ((pos = cppstring.find(delimiter)) == std::string::npos) {
			std::cerr << "Error parsin the color for the string: " << string << std::endl;
			return;
		}

		token = cppstring.substr(0, pos);
		array[i] = atoi(token.c_str());
		cppstring.erase(0, pos + delimiter.length());
	}
	array[2] = atoi(cppstring.c_str());


	(*color) = cv::Scalar(array[0], array[1], array[2]);
}

