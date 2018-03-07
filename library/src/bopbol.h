#ifndef BOPBOL_H_
#define BOPBOL_H_

extern "C" {

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

	BB_DEFINE_HANDLE(BbInstance);

	typedef int(__stdcall *BB_COORDINATE_CALLBACK)(float, float);

	typedef int(__stdcall *BB_ERROR_CALLBACK)(int);


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

	//
	//	Functions to create and destroy the local state of the library
	//	from outside.
	//
	IMAGE_DLL_API BbInstance bbCreateInstance();
	IMAGE_DLL_API void bbDestroyInstance(BbInstance instance);

	//
	//	Function used to check if the outside program is able to call this DLL
	//	correctly,
	//
	IMAGE_DLL_API bool bbIsCallable();

	//
	//	Initiates all the variables in the state necessary for the image processing
	//
	IMAGE_DLL_API BbResult bbInit(BbInstance instance);

	//
	//	Functions to both launch and stop the processing of the images
	//
	IMAGE_DLL_API BbResult bbLaunch(BbInstance instance);
	IMAGE_DLL_API BbResult bbStop(BbInstance instance);

	//
	//	Functions to configure local state
	//
	IMAGE_DLL_API BbResult bbSetBallHSVRanges(
		BbInstance instance,
		int iLowH, int iHighH, int iLowS,
		int iHighS, int iLowV, int iHighV);

	IMAGE_DLL_API BbResult bbSetBallRadiusThreshold(
		BbInstance instance,
		int radius);

	IMAGE_DLL_API BbResult bbSetConfigurationParameters(
		BbInstance instance,
		int show_collisions,
		bool using_video_file,
		bool show_trackbars,
		bool output_frames);


	//
	//	Functions to set up the callbacks for all the different actions
	//
	IMAGE_DLL_API BbResult bbSetCoordinateCallback(
		BbInstance instance,
		BB_COORDINATE_CALLBACK callback_function_ptr);

	IMAGE_DLL_API BbResult bbSetErrorCallback(
		BbInstance instance,
		BB_ERROR_CALLBACK callback_function_ptr);


	//
	//	Functions dedicated to calibration
	//

	IMAGE_DLL_API BbResult bbStartAreaCalibration(
		BbInstance instance);

	IMAGE_DLL_API BbAreaCalibration bbEndAreaCalibration(
		BbInstance instance);

	IMAGE_DLL_API BbResult bbCalibrateAreaWithClick(
		BbInstance instance,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	IMAGE_DLL_API BbResult bbCalibrateAreaWithHSVRanges(
		BbInstance instance,
		int iLowH, int iHighH, int iLowS,
		int iHighS, int iLowV, int iHighV);

	IMAGE_DLL_API BbResult bbCalibrateBallWithClick(
		BbInstance instance,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	//
	//	Functions to save and load calibration settings
	//

	IMAGE_DLL_API BbCalibrationSettings bbGetCalibrationSettings(BbInstance instance);

	IMAGE_DLL_API BbResult bbSetCalibrationSettings(BbInstance instance, BbCalibrationSettings calibration_settings);

}
#endif