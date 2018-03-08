#ifndef BOPBOL_H_
#define BOPBOL_H_

extern "C" {

/**
Including <cstdint> to hold the type uint32_t used for the version of this file
*/
#include <cstdint>

#define BB_MAKE_VERSION(major, minor, patch) (((major) << 22) | ((minor) << 12) | (patch))
#define BB_VERSION BB_MAKE_VERSION(1,0,0)

#define BB_VERSION_MAJOR(version) ((uint32_t)(version) >> 22)
#define BB_VERSION_MINOR(version) (((uint32_t)(version) >> 12) & 0x3ff)
#define BB_VERSION_PATCH(version) ((uint32_t)(version) & 0xfff)


#define IMAGE_DLL_API __declspec(dllexport)

#define BB_DEFINE_HANDLE(object) typedef struct object##_T* object

	enum BbResult {
		BB_SUCCESS = 0,
		BB_FAILURE = -1,
	};

	enum BbError {
		DOIN_GOOD,
		UNABLE_TO_OPEN_VIDEO,
		COULD_NOT_READ_FRAME,
		NOT_IN_CALIBRATION_MODE,
		COULD_NOT_CALIBRATE
	};

	BB_DEFINE_HANDLE(BbInstance);

	/**
	Type of the callback function that will be called when a collision of the ball
	against the area is detected.

	@param The normalized X coordinate (from 0 to 1) of the collision in the area
	@param The normalized Y coordinate (from 0 to 1) of the collision in the area
	@return Any integer value used to indicate status, currently unused.
	*/
	typedef int(__stdcall *BbCoordinateCallback)(float, float);

	/**
	Type of the callback function that will be called when an error has happened

	@param The error code from BbError indicating the error type.
	@return Any integer value used to indicate status, currently unused.
	@see BbError
	*/
	typedef int(__stdcall *BbErrorCallback)(int);


	struct BbPoint2d {
		double x = 0;
		double y = 0;
	};

	struct BbAreaCalibration {
		BbPoint2d point_0;
		BbPoint2d point_1;
		BbPoint2d point_2;
		BbPoint2d point_3;
		bool valid = false;
	};

	struct BbBallDetectionParameters {

		//
		//	Variables Storing HSV ranges for the
		//	ball detection. Defaults are for the tennis ball
		//
		int h_low = 23;
		int h_high = 43;
		int s_low = 30;
		int s_high = 255;
		int v_low = 50;
		int v_high = 255;

		//
		//	Just a threshold for calculating the min radius that
		//	we will consider a circle, pretty arbitrary for now but
		//	does the job pretty well. #MagicNumbers
		//
		int radius_threshold = 4;


	};

	struct BbCalibrationSettings {
		BbAreaCalibration projection_calibration;
		BbBallDetectionParameters ball_detection_parameters;
	};

	/**
	Creates an instance of this library to use to detect the ball collisions

	@return A valid BbInstance that has NOT been initialized
	@see bbInit
	*/
	IMAGE_DLL_API BbInstance bbCreateInstance();

	/**
	Destroys an instance of this library

	@param A valid BbInstance to be destroyed
	*/
	IMAGE_DLL_API void bbDestroyInstance(BbInstance instance);

	/**
	Function used to check if the outside program is able to call this DLL correctly.

	@return true if the library is correctly linked and called
	*/
	IMAGE_DLL_API bool bbIsCallable(uint32_t version = BB_VERSION);

	/**
	Initiates all the variables in the state necessary for the image processing

	@param the BbInstance to be initiated
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbInit(BbInstance instance);

	/**
	Launches processing of the webcam images

	@param the BbInstance that will launch the image processing
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbLaunch(BbInstance instance);

	/**
	Stops processing of the webcam images

	@param the BbInstance that will stop processing the image
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbStop(BbInstance instance);


	/**
	Sets the hue, saturation and value ranges to detect the ball being thrown
	
	@param the BbInstance that will hold the configuration parameters
	@param the lower range for the hue
	@param the upper range for the hue
	@param the lower range for the saturation
	@param the upper range for the saturation
	@param the lower range for the value
	@param the upper range for the value
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbSetBallHSVRanges(
		BbInstance instance,
		int a_low_h, int a_high_h,
		int a_low_s, int a_high_s,
		int a_low_v, int a_high_v);

	/**
	Sets radius threshold to detect the ball being thrown

	@param the BbInstance that will hold the configuration parameters
	@param the radius of the ball to be detected
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbSetBallRadiusThreshold(
		BbInstance instance,
		int radius);

	/**
	Sets library configurations parameters for the library

	@param the BbInstance that will hold the configuration parameters
	@param 1 if we want to show the collision where the ball collided in the frame window
	@param boolean indicatin if we should use a video file to read the image from
	@param boolean indicating if we should show the trackbars to change configuration on the fly
	@param boolean indicating if we should visualize the frames being parsed in real time
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbSetConfigurationParameters(
		BbInstance instance,
		int show_collisions,
		bool using_video_file,
		bool show_trackbars,
		bool output_frames);

	/**
	Sets the callback that will be called when we detect a ball collision.

	@param the BbInstance that will call the collision function
	@param the Callback function to be called with the collision data
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	@see BbCoordinateCallback
	*/
	IMAGE_DLL_API BbResult bbSetCoordinateCallback(
		BbInstance instance,
		BbCoordinateCallback callback_function_ptr);

	/**
	Sets the callback that will be called when we detect an error.

	@param the BbInstance that will call the error function
	@param the Callback function to be called with the error data
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	@see BbErrorCallback
	*/
	IMAGE_DLL_API BbResult bbSetErrorCallback(
		BbInstance instance,
		BbErrorCallback callback_function_ptr);

	/**
	Starts the area calibration period, should be called before the
	bbCalibrateArea* functions

	@param the BbInstance that we want to calibrate
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbStartAreaCalibration(
		BbInstance instance);

	/**
	Ends the area calibration period, should be called after the
	bbCalibrateArea* functions

	@param the BbInstance that we want to calibrate
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbAreaCalibration bbEndAreaCalibration(
		BbInstance instance);

	/**
	Calibrates the area that will receive the ball collisions by clicking on it in
	a popup window. It will use the hsv thresholds passed to detect the range of colours
	of said area.

	@param the BbInstance that we want to calibrate
	@param threshold value for the hue of the area
	@param threshold value for the saturation of the area
	@param threshold value for the value of the area
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbCalibrateAreaWithClick(
		BbInstance instance,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	/**
	Calibrates the area that will receive the ball collisions by sending the hsv
	ranges of the colour of the area and calculating the biggest rect we can find.

	@param the BbInstance that we want to calibrate
	@param the lower range for the hue
	@param the upper range for the hue
	@param the lower range for the saturation
	@param the upper range for the saturation
	@param the lower range for the value
	@param the upper range for the value
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbCalibrateAreaWithHSVRanges(
		BbInstance instance,
		int a_low_h, int a_high_h,
		int a_low_s, int a_high_s,
		int a_low_v, int a_high_v);

	/**
	Calibrates the ball that will be thrown by clicking on it in
	its darkest and lightest colors in two popup windows. It will use the
	hsv thresholds passed to detect the range of colours of the ball.

	@param the BbInstance that we want to calibrate
	@param threshold value for the hue of the ball
	@param threshold value for the saturation of the ball
	@param threshold value for the value of the ball
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbCalibrateBallWithClick(
		BbInstance instance,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	/**
	Returns the structure with the current calibration settings so it can
	be stored and loaded with the bbSetCalibrationSettings function in the 
	future without the need to go through the recalibration process.

	@param the BbInstance that we want to retrieve the calibration settings from
	@return BbCalibrationSettings structure with the current calibration settings
	*/
	IMAGE_DLL_API BbCalibrationSettings bbGetCalibrationSettings(BbInstance instance);

	/**
	Loads the calibration settings stored in the structure retrieved in the past
	with the bbSetCalibrationSettings function so we don't need to go through
	the recalibration process again.

	@param the BbInstance that we want to set the calibration settings of
	@param BbCalibrationSettings structure with the calibration settings to load
	@return BbResult indicating success (BB_SUCCESS) or an error with its code from the enum BbResult
	*/
	IMAGE_DLL_API BbResult bbSetCalibrationSettings(BbInstance instance, BbCalibrationSettings calibration_settings);

}
#endif