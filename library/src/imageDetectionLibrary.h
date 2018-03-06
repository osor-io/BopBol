#define IMAGE_DLL_API __declspec(dllexport)

#ifndef STATE_TYPE
#define STATE_TYPE void*
#endif // !STATE_TYPE



typedef int(__stdcall *COORDINATE_CALLBACK)(float, float);

typedef int(__stdcall *ERROR_CALLBACK)(int);


extern "C" {

#ifndef DATA_STRUCTURES
#define DATA_STRUCTURES

	struct SinglePoint {
		double x = 0;
		double y = 0;
	};

	struct ProjectionCalibration {
		SinglePoint point_0;
		SinglePoint point_1;
		SinglePoint point_2;
		SinglePoint point_3;
		bool valid = false;
	};

	struct BallDetectionParameters {

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

	struct CalibrationSettings {
		ProjectionCalibration projection_calibration;
		BallDetectionParameters ball_detection_parameters;
	};
#endif

	//
	//	Functions to create and destroy the local state of the library
	//	from outside.
	//
	IMAGE_DLL_API STATE_TYPE UNITY_createLibraryState();
	IMAGE_DLL_API void UNITY_destroyLibraryState(STATE_TYPE library_state);

	//
	//	Function used to check if the outside program is able to call this DLL
	//	correctly,
	//
	IMAGE_DLL_API bool UNITY_isAbleToConnectToDLL();

	//
	//	Initiates all the variables in the state necessary for the image processing
	//
	IMAGE_DLL_API int UNITY_initImageDetection(STATE_TYPE library_state);

	//
	//	Functions to both launch and stop the processing of the images
	//
	IMAGE_DLL_API int UNITY_launchImageProcessing(STATE_TYPE library_state);
	IMAGE_DLL_API int UNITY_stopImageDetection(STATE_TYPE library_state);

	//
	//	Functions to configure local state
	//
	IMAGE_DLL_API int UNITY_setBall_HSVRanges(
		STATE_TYPE library_state,
		int iLowH, int iHighH, int iLowS,
		int iHighS, int iLowV, int iHighV);

	IMAGE_DLL_API int UNITY_setBall_RadiusThreshold(
		STATE_TYPE library_state,
		int radius);

	IMAGE_DLL_API int UNITY_setConfigurationParameters(
		STATE_TYPE library_state,
		int show_collisions,
		bool using_video_file,
		bool show_trackbars,
		bool output_frames);


	//
	//	Functions to set up the callbacks to Unity
	//
	IMAGE_DLL_API int UNITY_setCoordinateCallback(
		STATE_TYPE library_state,
		COORDINATE_CALLBACK callback_function_ptr);

	IMAGE_DLL_API int UNITY_setErrorCallback(
		STATE_TYPE library_state,
		ERROR_CALLBACK callback_function_ptr);


	//
	//	Functions dedicated to calibration
	//

	IMAGE_DLL_API int UNITY_startProjectionCalibration(
		STATE_TYPE library_state);

	IMAGE_DLL_API ProjectionCalibration UNITY_endProjectionCalibration(
		STATE_TYPE library_state);

	IMAGE_DLL_API int UNITY_calibrateProjectionWithClick(
		STATE_TYPE library_state,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	IMAGE_DLL_API int UNITY_calibrateProjectionWithHSVRanges(
		STATE_TYPE library_state,
		int iLowH, int iHighH, int iLowS,
		int iHighS, int iLowV, int iHighV);

	IMAGE_DLL_API int UNITY_calibrateBallWithClick(
		STATE_TYPE library_state,
		int hue_threshold,
		int saturation_threshold,
		int value_threshold);

	//
	//	Functions to save and load calibration settings
	//

	IMAGE_DLL_API CalibrationSettings UNITY_getCalibrationSettings(STATE_TYPE library_state);

	IMAGE_DLL_API int UNITY_setCalibrationSettings(STATE_TYPE library_state, CalibrationSettings calibration_settings);

}