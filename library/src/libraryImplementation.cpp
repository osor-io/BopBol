#include "libraryImplementation.h"
#include <thread>
#include <chrono>

using namespace cv;
using namespace std;

//
//	============================================
//				MAIN IMPLEMENTATION
//	============================================
//

STATE_TYPE createLibraryState() {

	LibraryState * state_ptr = new LibraryState();

	if (state_ptr == NULL) {
		return NULL;
	}

	state_ptr->s_video = new VideoCapture();

	return (STATE_TYPE)state_ptr;
}


void destroyLibraryState(STATE_TYPE library_state) {

	LibraryState* state_ptr = (LibraryState*)library_state;

	delete state_ptr->s_video;

	if (library_state != NULL) {
		free(library_state);
	}
}


int initImageDetection(STATE_TYPE library_state) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();


	//
	//	This is our circular queue that will store the
	//	last DEQUE_LENGTH positions of the ball in the frame 
	//
	deque_init(&state->s_main_deque);

	//
	//	Simply opening the video source for the webcam since
	//	we are always calling from Unity
	//
	state->s_video->open(0);



	//
	// We check if we can read the video
	//
	if (!state->s_video->isOpened()) {
		cout << "Could not read video file" << endl;
		manageError(state->s_callback_functions.error_callback, ERROR::UNABLE_TO_OPEN_VIDEO);

		state->s_configuration_mutex.unlock();
		return FAILURE;
	}

	state->s_video->release();


	state->s_configuration_mutex.unlock();


	return SUCCESS;
}


int launchImageProcessing(STATE_TYPE library_state) {


	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	//
	//	Inside this function we are opening and releasing
	//	the video source to avoid accessing the webcam constantly.
	//
	state->s_video->open(0);

	state->s_should_stop = false;
	state->s_running = true;

	//
	//	We destroy the previous windows that might me mangling
	//	arround.
	//
	destroyAllWindows();

	//
	//	And we show the trackbars to give the option to change
	//	the parameters to detect the ball.
	//
	if (state->s_configuration_parameters.show_trackbars) {

		//
		//	We give the option to change the hsv ranges with
		//	trackbars
		//
		cv::namedWindow("Control", CV_WINDOW_FREERATIO);
		cvCreateTrackbar("LowH", "Control", &state->s_ball_detection_parameters.h_low, 179); //Hue (0 - 179)
		cvCreateTrackbar("HighH", "Control", &state->s_ball_detection_parameters.h_high, 179);
		cvCreateTrackbar("LowS", "Control", &state->s_ball_detection_parameters.s_low, 255); //Saturation (0 - 255)
		cvCreateTrackbar("HighS", "Control", &state->s_ball_detection_parameters.s_high, 255);
		cvCreateTrackbar("LowV", "Control", &state->s_ball_detection_parameters.v_low, 255); //Value (0 - 255)
		cvCreateTrackbar("HighV", "Control", &state->s_ball_detection_parameters.v_high, 255);
		cvCreateTrackbar("Radius", "Control", &state->s_ball_detection_parameters.radius_threshold, 100);
		cvCreateTrackbar("Contour Limit", "Control", &state->s_contour_parameters.circle_contour_limit, 20);
		cvCreateTrackbar("Epsilon Multiplier", "Control", &state->s_contour_parameters.epsilon_multiplier, 200);
		cvCreateTrackbar("Collision", "Control", &state->s_configuration_parameters.show_collisions, 1);
	}

	while (!state->s_should_stop) {

		//
		//	For the time being we will just lock the configuration for a whole frame
		//	since we will not be changing it during processing very ofter (or ever).
		//
		state->s_configuration_mutex.lock();
		parse_frame(library_state);
		state->s_configuration_mutex.unlock();

	}

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	destroyAllWindows();

	state->s_video->release();

	state->s_running = false;

	return SUCCESS;
}


int stopImageDetection(STATE_TYPE library_state) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_should_stop = true;

	while (state->s_running) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	destroyAllWindows();

	return SUCCESS;
}


int setBall_HSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	state->s_ball_detection_parameters.h_low = iLowH;
	state->s_ball_detection_parameters.h_high = iHighH;
	state->s_ball_detection_parameters.s_low = iLowS;
	state->s_ball_detection_parameters.s_high = iHighS;
	state->s_ball_detection_parameters.v_low = iLowV;
	state->s_ball_detection_parameters.v_high = iHighV;

	state->s_configuration_mutex.unlock();


	return SUCCESS;
}


int setBall_RadiusThreshold(
	STATE_TYPE library_state,
	int radius) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	state->s_ball_detection_parameters.radius_threshold = radius;

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}


int setConfigurationParameters(
	STATE_TYPE library_state,
	int show_collisions,
	bool using_video_file,
	bool show_trackbars,
	bool output_frames) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	state->s_configuration_parameters.show_collisions = show_collisions;
	state->s_configuration_parameters.using_video_file = using_video_file;
	state->s_configuration_parameters.show_trackbars = show_trackbars;
	state->s_configuration_parameters.output_frames = output_frames;

	state->s_configuration_mutex.unlock();

	return SUCCESS;

}

int setCoordinateCallback(
	STATE_TYPE library_state,
	COORDINATE_CALLBACK callback_function_ptr) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	state->s_callback_functions.coordinate_callback = callback_function_ptr;

	state->s_configuration_mutex.unlock();

	return SUCCESS;

}

int setErrorCallback(
	STATE_TYPE library_state,
	ERROR_CALLBACK callback_function_ptr) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	state->s_callback_functions.error_callback = callback_function_ptr;

	state->s_configuration_mutex.unlock();

	return SUCCESS;

}

int startProjectionCalibration(
	STATE_TYPE library_state) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	//
	//	Inside this function we are opening and releasing
	//	the video source to avoid accessing the webcam constantly.
	//
	state->s_video->open(0);

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	destroyAllWindows();

	state->s_is_calibrating_projection = true;

	state->s_calibration_state.have_matrix = false;

	state->s_calibration_state.square_points.clear();

	state->s_calibration_state.average_points.clear();

	state->s_calibration_state.homography_matrix = Mat();

	//
	//	@@TODO: Complete and probably erase the previous values
	//	that represent the calibration (Homography matrices, 
	//	points that define the squares, etc.).
	//

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}

ProjectionCalibration endProjectionCalibration(
	STATE_TYPE library_state) {

	if (library_state == NULL) return ProjectionCalibration();
	LibraryState* state = (LibraryState*)library_state;

	vector<Point2f> normal_values;
	normal_values.push_back(Point2f(0.f, 1.f));
	normal_values.push_back(Point2f(1.f, 1.f));
	normal_values.push_back(Point2f(1.f, 0.f));
	normal_values.push_back(Point2f(0.f, 0.f));


	state->s_configuration_mutex.lock();


	if (state->s_calibration_state.square_points.size() <= 0) {

		manageError(state->s_callback_functions.error_callback, ERROR::COULD_NOT_CALIBRATE);

		destroyAllWindows();

		state->s_video->release();

		state->s_is_calibrating_projection = false;

		state->s_calibration_state.have_matrix = true;
		state->s_calibration_state.average_points.clear();
		state->s_calibration_state.square_points.clear();
		state->s_calibration_state.homography_matrix = Mat();

		state->s_configuration_mutex.unlock();

		return ProjectionCalibration();

	}

	state->s_calibration_state.average_points.clear();
	state->s_calibration_state.average_points.push_back(Point2f(0.f, 0.f));
	state->s_calibration_state.average_points.push_back(Point2f(0.f, 0.f));
	state->s_calibration_state.average_points.push_back(Point2f(0.f, 0.f));
	state->s_calibration_state.average_points.push_back(Point2f(0.f, 0.f));


	for (vector<Point2f> vec : state->s_calibration_state.square_points) {

		for (short i = 0; i < 4; i++) {
			state->s_calibration_state.average_points[i] += vec[i];
		}

	}

	for (short i = 0; i < 4; i++) {
		state->s_calibration_state.average_points[i] /= (float)state->s_calibration_state.square_points.size();
	}

	utilscv_sortSquarePoints(&(state->s_calibration_state.average_points));

	ProjectionCalibration projection_calibration;

	projection_calibration.point_0.x = state->s_calibration_state.average_points[0].x;
	projection_calibration.point_0.y = state->s_calibration_state.average_points[0].y;

	projection_calibration.point_1.x = state->s_calibration_state.average_points[1].x;
	projection_calibration.point_1.y = state->s_calibration_state.average_points[1].y;

	projection_calibration.point_2.x = state->s_calibration_state.average_points[2].x;
	projection_calibration.point_2.y = state->s_calibration_state.average_points[2].y;

	projection_calibration.point_3.x = state->s_calibration_state.average_points[3].x;
	projection_calibration.point_3.y = state->s_calibration_state.average_points[3].y;


	state->s_calibration_state.homography_matrix = findHomography(state->s_calibration_state.average_points, normal_values);

	//
	//	@@TODO: Complete and calculate our final values
	//	for the final Homography matrix. Those should be
	//	calculated taking all the values obtained in the
	//	different calibration for all the colours.
	//
	//	Obviously, we will need some kind of method
	//	to discard very different points caused by a bad calibration
	//	because of the chosen color.
	//

	//
	//	We destroy all windows to get ready for a possible
	//	next execution
	//
	destroyAllWindows();

	state->s_video->release();

	state->s_is_calibrating_projection = false;

	state->s_calibration_state.have_matrix = true;

	state->s_configuration_mutex.unlock();

	return projection_calibration;

}


struct MouseClick {
	cv::Point2i point;
	bool clicked = false;
};

//
//	Reads the position of the mouse and returns it in the struct
//
void onMouse(int event, int x, int y, int flags, void* param) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		MouseClick* data = (MouseClick*)(param);
		data->point.x = x;
		data->point.y = y;
		data->clicked = true;
	}
}

int calibrateProjectionWithClick(
	STATE_TYPE library_state,
	int hue_threshold, int saturation_threshold, int value_threshold) {

	Mat clean_frame, frame, mask;

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	if (!state->s_is_calibrating_projection) {
		manageError(state->s_callback_functions.error_callback, ERROR::NOT_IN_CALIBRATION_MODE);
		state->s_configuration_mutex.unlock();
		return FAILURE;
	}


	//
	//	We wait a bit to have a clear frame
	//
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (short i = 0; i < CALIBRATION_WARMUP; i++) {
		if (!state->s_video->read(clean_frame)) {
			manageError(state->s_callback_functions.error_callback, ERROR::COULD_NOT_READ_FRAME);
			return FAILURE;
		}
		waitKey(1);
	}


	Vec3b hsv_base;
	MouseClick click;
	while (state->s_video->read(clean_frame)) {
		imshow("Click on the projection", clean_frame);
		setMouseCallback("Click on the projection", onMouse, &click);
		if (click.clicked) {
			Vec3b rgb = clean_frame.at<Vec3b>((click.point));
			Mat HSV;
			Mat RGB = clean_frame(Rect(click.point.x, click.point.y, 1, 1));
			cvtColor(RGB, HSV, CV_BGR2HSV);
			hsv_base = HSV.at<Vec3b>(0, 0);
			break;
		}
		int k = waitKey(10);
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
	utilscv_resize(&clean_frame, state->s_configuration_parameters.target_internal_resolution);


	//
	//	Change the frame to HSV color format
	//
	cvtColor(clean_frame, frame, CV_BGR2HSV);


	//
	//	And we get our frame with only the ball's pixels, we do some
	//	erosions and dilations to avout bumps and stuff
	//
	cv::inRange(frame,
		Scalar(hsv_base[0] - hue_threshold,
			hsv_base[1] - saturation_threshold,
			hsv_base[2] - value_threshold),
		Scalar(hsv_base[0] + hue_threshold,
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
		vector<vector<Point>> contours;
		vector<Point2f> processed_contour;
		bool found_valid_contour = false;
		vector<Vec4i> hierarchy;

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

				vector<Point> aux_processed_contour;

				//
				//	Now we use approxPoly to get the processed rectangle
				//	
				approxPolyDP(Mat(contours[i]), aux_processed_contour, 3, true);

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

				if (state->s_configuration_parameters.output_frames) {
					cv::circle(clean_frame, processed_contour[0], 3, Scalar(255, 0, 0), -1);
					cv::circle(clean_frame, processed_contour[1], 3, Scalar(0, 255, 0), -1);
					cv::circle(clean_frame, processed_contour[2], 3, Scalar(255, 0, 255), -1);
					cv::circle(clean_frame, processed_contour[3], 3, Scalar(0, 0, 255), -1);
				}

				//
				//	We sabe the calibration points for this iteration of the calibration
				//
				state->s_calibration_state.square_points.push_back(processed_contour);
			}


		}

		//
		//	Finally we can show the frames if we want
		//
		if (state->s_configuration_parameters.output_frames) {
			imshow("clean_frame", clean_frame);
			imshow("mask", mask);
		}

		waitKey(1);

	}

	//
	//	ANd we wait a bit after the calibration to be able to see
	//	the frames if needed
	//
	if (state->s_configuration_parameters.output_frames) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	else {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}

int calibrateProjectionWithHSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {

	Mat clean_frame, frame, mask;

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	if (!state->s_is_calibrating_projection) {
		manageError(state->s_callback_functions.error_callback, ERROR::NOT_IN_CALIBRATION_MODE);
		state->s_configuration_mutex.unlock();
		return FAILURE;
	}


	//
	//	We wait a bit to have a clear frame
	//
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	for (short i = 0; i < CALIBRATION_WARMUP; i++) {
		if (!state->s_video->read(clean_frame)) {
			manageError(state->s_callback_functions.error_callback, ERROR::COULD_NOT_READ_FRAME);
			return FAILURE;
		}
		waitKey(1);
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
	utilscv_resize(&clean_frame, state->s_configuration_parameters.target_internal_resolution);


	//
	//	Change the frame to HSV color format
	//
	cvtColor(clean_frame, frame, CV_BGR2HSV);


	//
	//	And we get our frame with only the ball's pixels, we do some
	//	erosions and dilations to avout bumps and stuff
	//
	cv::inRange(frame,
		Scalar(iLowH,
			iLowS,
			iLowV),
		Scalar(iHighH,
			iHighS,
			iHighV),
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
		vector<vector<Point>> contours;
		vector<Point2f> processed_contour;
		bool found_valid_contour = false;
		vector<Vec4i> hierarchy;

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

				vector<Point> aux_processed_contour;

				//
				//	Now we use approxPoly to get the processed rectangle
				//	
				approxPolyDP(Mat(contours[i]), aux_processed_contour, 3, true);

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

				if (state->s_configuration_parameters.output_frames) {
					cv::circle(clean_frame, processed_contour[0], 3, Scalar(255, 0, 0), -1);
					cv::circle(clean_frame, processed_contour[1], 3, Scalar(0, 255, 0), -1);
					cv::circle(clean_frame, processed_contour[2], 3, Scalar(255, 0, 255), -1);
					cv::circle(clean_frame, processed_contour[3], 3, Scalar(0, 0, 255), -1);
				}

				//
				//	We sabe the calibration points for this iteration of the calibration
				//
				state->s_calibration_state.square_points.push_back(processed_contour);
			}


		}

		//
		//	Finally we can show the frames if we want
		//
		if (state->s_configuration_parameters.output_frames) {
			imshow("clean_frame", clean_frame);
			imshow("mask", mask);
		}

		waitKey(1);

	}

	//
	//	ANd we wait a bit after the calibration to be able to see
	//	the frames if needed
	//
	if (state->s_configuration_parameters.output_frames) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	else {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

	}

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}

void reorderBallRanges(LibraryState* state) {

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

int calibrateBallWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {

	Mat clean_frame, frame, mask;

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();
	{

		state->s_video->open(0);

		//
		//	We wait a bit to have a clear frame
		//
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		for (short i = 0; i < CALIBRATION_WARMUP; i++) {
			if (!state->s_video->read(clean_frame)) {
				manageError(state->s_callback_functions.error_callback, ERROR::COULD_NOT_READ_FRAME);
				return FAILURE;
			}
			waitKey(1);
		}


		Vec3b hsv_base_low;
		MouseClick click_low;
		while (state->s_video->read(clean_frame)) {
			imshow("Click on dark version of the ball", clean_frame);
			setMouseCallback("Click on dark version of the ball", onMouse, &click_low);
			if (click_low.clicked) {
				Vec3b rgb = clean_frame.at<Vec3b>((click_low.point));
				Mat HSV;
				Mat RGB = clean_frame(Rect(click_low.point.x, click_low.point.y, 1, 1));
				cvtColor(RGB, HSV, CV_BGR2HSV);
				hsv_base_low = HSV.at<Vec3b>(0, 0);
				break;
			}
			int k = waitKey(10);
		}

		Vec3b hsv_base_high;
		MouseClick click_high;
		while (state->s_video->read(clean_frame)) {
			imshow("Click on lit version of the ball", clean_frame);
			setMouseCallback("Click on lit version of the ball", onMouse, &click_high);
			if (click_high.clicked) {
				Vec3b rgb = clean_frame.at<Vec3b>((click_high.point));
				Mat HSV;
				Mat RGB = clean_frame(Rect(click_high.point.x, click_high.point.y, 1, 1));
				cvtColor(RGB, HSV, CV_BGR2HSV);
				hsv_base_high = HSV.at<Vec3b>(0, 0);
				break;
			}
			int k = waitKey(10);
		}

		state->s_ball_detection_parameters.h_low = hsv_base_low[0];
		state->s_ball_detection_parameters.s_low = hsv_base_low[1];
		state->s_ball_detection_parameters.v_low = hsv_base_low[2];

		state->s_ball_detection_parameters.h_high = hsv_base_high[0];
		state->s_ball_detection_parameters.s_high = hsv_base_high[1];
		state->s_ball_detection_parameters.v_high = hsv_base_high[2];

		reorderBallRanges(state);

		state->s_ball_detection_parameters.h_low -= hue_threshold;
		state->s_ball_detection_parameters.s_low -= saturation_threshold;
		state->s_ball_detection_parameters.v_low -= value_threshold;

		state->s_ball_detection_parameters.h_high += hue_threshold;
		state->s_ball_detection_parameters.s_high += saturation_threshold;
		state->s_ball_detection_parameters.v_high += value_threshold;

		//
		//	@@DOING: Setting saturation and value to broad ranges
		//
		state->s_ball_detection_parameters.s_low = 100;
		state->s_ball_detection_parameters.s_high = 255;
		state->s_ball_detection_parameters.v_low = 30;
		state->s_ball_detection_parameters.v_high = 255;


		state->s_video->release();

		destroyAllWindows();
	}

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}

int calibrateBall_highRange(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {

	//
	//	@@TODO
	//

	return SUCCESS;
}

CalibrationSettings getCalibrationSettings(
	STATE_TYPE library_state) {

	CalibrationSettings calibration_settings;


	if (library_state == NULL) return CalibrationSettings();
	LibraryState* state = (LibraryState*)library_state;

	state->s_configuration_mutex.lock();

	{

		ProjectionCalibration projection_calibration_aux;

		if (state->s_calibration_state.have_matrix && state->s_calibration_state.average_points.size() > 0) {
			//
			//	Setting the points that define the square on the screen
			//
			projection_calibration_aux.point_0.x = state->s_calibration_state.average_points[0].x;
			projection_calibration_aux.point_0.y = state->s_calibration_state.average_points[0].y;

			projection_calibration_aux.point_1.x = state->s_calibration_state.average_points[1].x;
			projection_calibration_aux.point_1.y = state->s_calibration_state.average_points[1].y;

			projection_calibration_aux.point_2.x = state->s_calibration_state.average_points[2].x;
			projection_calibration_aux.point_2.y = state->s_calibration_state.average_points[2].y;

			projection_calibration_aux.point_3.x = state->s_calibration_state.average_points[3].x;
			projection_calibration_aux.point_3.y = state->s_calibration_state.average_points[3].y;

			projection_calibration_aux.valid = true;

			calibration_settings.projection_calibration = projection_calibration_aux;
		}
		else {
			projection_calibration_aux.valid = false;
		}

		//
		//	Setting the ball parameters
		//
		calibration_settings.ball_detection_parameters = state->s_ball_detection_parameters;
	}

	state->s_configuration_mutex.unlock();

	return calibration_settings;
}

int setCalibrationSettings(
	STATE_TYPE library_state,
	CalibrationSettings calibration_settings) {

	if (library_state == NULL) return FAILURE;
	LibraryState* state = (LibraryState*)library_state;

	vector<Point2f> normal_values;
	normal_values.push_back(Point2f(0.f, 1.f));
	normal_values.push_back(Point2f(1.f, 1.f));
	normal_values.push_back(Point2f(1.f, 0.f));
	normal_values.push_back(Point2f(0.f, 0.f));

	state->s_configuration_mutex.lock();

	{

		state->s_ball_detection_parameters = calibration_settings.ball_detection_parameters;

		if (calibration_settings.projection_calibration.valid) {

			ProjectionCalibration projection_calibration_aux = calibration_settings.projection_calibration;

			state->s_calibration_state.square_points.clear();

			state->s_calibration_state.average_points.clear();
			state->s_calibration_state.average_points.resize(4);

			state->s_calibration_state.average_points[0].x = (float)projection_calibration_aux.point_0.x;
			state->s_calibration_state.average_points[0].y = (float)projection_calibration_aux.point_0.y;

			state->s_calibration_state.average_points[1].x = (float)projection_calibration_aux.point_1.x;
			state->s_calibration_state.average_points[1].y = (float)projection_calibration_aux.point_1.y;

			state->s_calibration_state.average_points[2].x = (float)projection_calibration_aux.point_2.x;
			state->s_calibration_state.average_points[2].y = (float)projection_calibration_aux.point_2.y;

			state->s_calibration_state.average_points[3].x = (float)projection_calibration_aux.point_3.x;
			state->s_calibration_state.average_points[3].y = (float)projection_calibration_aux.point_3.y;

			state->s_calibration_state.homography_matrix = findHomography(state->s_calibration_state.average_points, normal_values);

			state->s_calibration_state.have_matrix = true;
		}
		else {
			state->s_calibration_state.have_matrix = false;
		}
	}

	state->s_configuration_mutex.unlock();

	return SUCCESS;
}







// ========================================
//		LOCAL FUNCTIONS IMPLEMENTATION
// ========================================

int parse_frame(STATE_TYPE library_state) {

	Mat clean_frame, frame, mask;

	LibraryState* state = (LibraryState*)library_state;


	if (!state->s_video->read(clean_frame)) {
		manageError(state->s_callback_functions.error_callback, ERROR::COULD_NOT_READ_FRAME);
		return FAILURE;
	}


	//
	//	We resize the frame to avoid tough computations
	//	and we still have to decide if we need to blur it
	//	the code is
	//		blur(frame, frame, Size(2, 2));
	//	But we won't do it for now
	//
	//	for resizing we either use
	utilscv_resize(&clean_frame, state->s_configuration_parameters.target_internal_resolution);
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
		Scalar(state->s_ball_detection_parameters.h_low,
			state->s_ball_detection_parameters.s_low,
			state->s_ball_detection_parameters.v_low),
		Scalar(state->s_ball_detection_parameters.h_high,
			state->s_ball_detection_parameters.s_high,
			state->s_ball_detection_parameters.v_high),
		mask);

	cv::erode(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);
	cv::dilate(mask, mask, Mat(), Point(-1, -1), 2, 1, 1);


	//
	//	We just care about the radius and the centre
	//	so we can open a new scope
	//
	Point2f center, centroid(-1.f, -1.f);
	float radius = 0.0f;
	bool found_circle = false;

	//
	//	@@SCOPE
	//	Here we find the properties of the circle that encloses the ball
	//
	{


#ifdef HOUGH

		vector<Vec3f> circles;

		Mat hough_frame = mask;


		cv::erode(hough_frame, hough_frame, Mat(), Point(-1, -1), 6, 1, 1);
		cv::dilate(hough_frame, hough_frame, Mat(), Point(-1, -1), 6, 1, 1);
		cv::GaussianBlur(hough_frame, hough_frame, cv::Size(9, 9), 2, 2);


		imshow("test blurred", hough_frame);
		waitKey(10);

		cv::HoughCircles(hough_frame, circles, CV_HOUGH_GRADIENT, 1, mask.rows / 8, 100, 20, 0, 0);

		for (size_t i = 0; i < circles.size(); i++)
		{
			//float radius_aux = cvRound(circles[i][2]);

			float radius_aux = circles[i][2];

			//
			//	We could probably remove this check since cv::HoughCircles is likely already doing it.
			//
			if (radius_aux > state->s_ball_detection_parameters.radius_threshold) {
				Point center_aux(cvRound(circles[i][0]), cvRound(circles[i][1]));
				radius = radius_aux;
				centroid = center_aux;
				found_circle = true;
			}

		}

#else	//We default to the contours method


		//
		//	Output Vectors for the contours
		//
		vector< vector<Point> > contours;
		vector<Vec4i> hierarchy;

		//
		//	We look for the external contours
		//
		cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		vector<vector<Point>> approx_contours;

		if (contours.size() > 0) {

			approx_contours.resize(contours.size());

			//
			//	And we take the biggest one, here we can decide to detect as
			//	many balls as needed calculating the N biggest contours
			//
			float largest_area = 0.0f;
			int largest_contour_index = -1;
			for (int i = 0; i < contours.size(); i++) {
				float epsilon = (float)state->s_contour_parameters.epsilon_multiplier / EPSILON_DIV * (float)arcLength(contours[i], true);
				approxPolyDP(contours[i], approx_contours[i], epsilon, true);
				float area = (float)contourArea(contours[i], false);

				if (area > largest_area && approx_contours[i].size() > state->s_contour_parameters.circle_contour_limit) {
					largest_area = area;
					largest_contour_index = i;
				}

			}

			if (state->s_configuration_parameters.output_frames) {
				drawContours(clean_frame, approx_contours, -1, Scalar(0, 255, 0));
			}


			if (largest_contour_index >= 0) {
				//
				//	And we take the minimum enclosing circle for our contour
				//	to be able to calculate its centroid
				//
				cv::minEnclosingCircle(contours[largest_contour_index], center, radius);
				Moments moments = cv::moments(contours[largest_contour_index]);
				centroid = Point2f((float)moments.m10 / (float)moments.m00, (float)moments.m01 / (float)moments.m00);

				if (radius > state->s_ball_detection_parameters.radius_threshold) {
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
		Scalar circle_color(255, 255, 0);
		Scalar centroid_color(255, 0, 0);
		int circle_thickness = 2;

		//
		//	We detect here if there has been a collision
		//
		if (found_circle && state->s_main_deque.size > DEQUE_SIZE_FOR_COLLISION && centroid.x > 0.0f) {
			//
			//	For the old direction we can calculate the average of the last N frames
			//
			float old_direction = deque_getElementAt(&state->s_main_deque, 1).x - deque_getElementAt(&state->s_main_deque, 1 + COLLISION_PAST_STEPS).x;
			float curr_direction = centroid.x - deque_getElementAt(&state->s_main_deque, 1).x;


			//
			//	We have a collision if
			//
			if (old_direction * curr_direction < 0.0f) {

				//
				//	So we update the collision coordinates and set up
				//	the amount of frames we want to show the collision for
				//
				state->s_last_collision_coordinates = deque_getElementAt(&state->s_main_deque, 1);

				//
				//	Correct for the depth of the ball
				//
				if (old_direction > 0) {
					state->s_last_collision_coordinates.x += radius * RADIUS_LATERAL_MULT;
				}
				else {
					state->s_last_collision_coordinates.x -= radius * RADIUS_LATERAL_MULT;
				}

				state->s_frames_remaining_collision = NUM_FRAMES_SHOW_COLLISION;


				if (state->s_calibration_state.have_matrix) {
					//
					//	Use OpenCV's findHomography and perspectiveTransform with
					//	the previously obtained Homography matrix to map screen coordinates
					//	to wall projection coordinates.
					//	
					vector<Point2f> input_not_transformed;
					input_not_transformed.push_back(state->s_last_collision_coordinates);
					vector<Point2f> output_transformed;
					output_transformed.push_back(Point2f(0, 0));
					perspectiveTransform(input_not_transformed, output_transformed, state->s_calibration_state.homography_matrix);

					//
					//	And we use the CALLBACK if we have to
					//
					if (state->s_callback_functions.coordinate_callback != NULL && !state->s_should_stop) {

						Point2f collision_transformed = output_transformed[0];

						state->s_callback_functions.coordinate_callback(
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
		if (found_circle && radius > state->s_ball_detection_parameters.radius_threshold) {

			if (state->s_configuration_parameters.show_collisions) {
				cv::circle(clean_frame, center, (int)radius, circle_color, circle_thickness);
				cv::circle(clean_frame, centroid, 3, centroid_color, -1);
			}

			//
			//	We insert the element in the deque
			//
			deque_insertElement(&state->s_main_deque, centroid);
			state->s_had_ball_previous_frame = true;
		}
		else {

			//
			//	@@DOING: Give a bit more leniance to detect the ball if
			//	we loose it for a few frames.
			//

			if (state->s_had_ball_previous_frame == true) {
				state->s_lost_ball_for_frames = NUM_FRAMES_LOST_BALL;
			}
			else if (state->s_lost_ball_for_frames > 0) {
				--state->s_lost_ball_for_frames;
			}
			else {
				state->s_lost_ball_for_frames = -1;
				deque_init(&state->s_main_deque);
			}

			state->s_had_ball_previous_frame = false;


		}
	}

	//
	//	And we print every line
	//
	if (state->s_main_deque.size > 0)
		for (unsigned int i = 0; i < state->s_main_deque.size - 1; i++) {
			cv::line(clean_frame, deque_getElementAt(&state->s_main_deque, i), deque_getElementAt(&state->s_main_deque, i + 1), Scalar(255, 0, 255));
		}

	//
	//	We also print here the last collision coordinates
	//	if we still have remaining frames for that
	//
	if (state->s_frames_remaining_collision > 0 && state->s_configuration_parameters.show_collisions) {
		state->s_frames_remaining_collision--;
		cv::circle(clean_frame, state->s_last_collision_coordinates, 10, Scalar(0, 0, 255), -1);
	}

	//
	//	We add the lines that define the projection rectangle
	//	with the perspective
	//
	if (state->s_configuration_parameters.show_collisions
		&& state->s_calibration_state.have_matrix
		&& state->s_calibration_state.average_points.size() > 0) {
		cv::line(clean_frame,
			state->s_calibration_state.average_points[0],
			state->s_calibration_state.average_points[1], Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			state->s_calibration_state.average_points[1],
			state->s_calibration_state.average_points[2], Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			state->s_calibration_state.average_points[2],
			state->s_calibration_state.average_points[3], Scalar(255, 100, 0), 4);
		cv::line(clean_frame,
			state->s_calibration_state.average_points[3],
			state->s_calibration_state.average_points[0], Scalar(255, 100, 0), 4);
	}

	//
	//	Finally we can show the frame
	//
	if (state->s_configuration_parameters.output_frames) {
		imshow("frame", clean_frame);
	}

	waitKey(1);

	return SUCCESS;

}

void showUsage() {
	cout << "\nThis program needs a source (the path) for the video, (or none for webcam)" << endl;
	cout << "EXAMPLES OF USAGE:"
		"\n\t With Video: $ executable.exe ./resources/video.mp4"
		"\n\t Without Video: $ executable.exe"
		<< endl;
	PRESS_TO_CONTINUE
}

void parseHSVColor(char* string, Scalar * color) {

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
			cerr << "Error parsin the color for the string: " << string << endl;
			return;
		}

		token = cppstring.substr(0, pos);
		array[i] = atoi(token.c_str());
		cppstring.erase(0, pos + delimiter.length());
	}
	array[2] = atoi(cppstring.c_str());


	(*color) = Scalar(array[0], array[1], array[2]);
}