#include "imageDetectionLibrary.h"
#include "libraryImplementation.h"

bool UNITY_isAbleToConnectToDLL() {
	return true;
}

int UNITY_initImageDetection(STATE_TYPE library_state) {
	return initImageDetection(library_state);
}

int UNITY_launchImageProcessing(STATE_TYPE library_state) {
	return launchImageProcessing(library_state);
}

int UNITY_stopImageDetection(STATE_TYPE library_state) {
	return stopImageDetection(library_state);
}

STATE_TYPE UNITY_createLibraryState() {
	return createLibraryState();
}

void UNITY_destroyLibraryState(STATE_TYPE library_state) {
	return destroyLibraryState(library_state);
}

int UNITY_setBall_HSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {
	return setBall_HSVRanges(library_state,
		iLowH, iHighH, iLowS,
		iHighS, iLowV, iHighV);
}

int UNITY_setBall_RadiusThreshold(
	STATE_TYPE library_state,
	int radius) {
	return setBall_RadiusThreshold(library_state, radius);
}

int UNITY_setConfigurationParameters(
	STATE_TYPE library_state,
	int show_collisions,
	bool using_video_file,
	bool show_trackbars,
	bool output_frames) {
	return setConfigurationParameters(library_state,
		show_collisions, using_video_file,
		show_trackbars, output_frames);
}

int UNITY_setCoordinateCallback(
	STATE_TYPE library_state,
	COORDINATE_CALLBACK callback_function_ptr) {
	return setCoordinateCallback(library_state, callback_function_ptr);
}

int UNITY_setErrorCallback(
	STATE_TYPE library_state,
	ERROR_CALLBACK callback_function_ptr) {
	return setErrorCallback(library_state, callback_function_ptr);
}


int UNITY_startProjectionCalibration(
	STATE_TYPE library_state) {
	return startProjectionCalibration(library_state);
}

ProjectionCalibration UNITY_endProjectionCalibration(
	STATE_TYPE library_state) {
	return endProjectionCalibration(library_state);
}

int UNITY_calibrateProjectionWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {
	return calibrateProjectionWithClick(library_state,
		hue_threshold,
		saturation_threshold,
		value_threshold);
}

int UNITY_calibrateProjectionWithHSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {
	return calibrateProjectionWithHSVRanges(library_state,
		iLowH, iHighH, iLowS,
		iHighS, iLowV, iHighV);
}

IMAGE_DLL_API int UNITY_calibrateBallWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {
	return calibrateBallWithClick(library_state,
		hue_threshold,
		saturation_threshold,
		value_threshold);
}


CalibrationSettings UNITY_getCalibrationSettings(STATE_TYPE library_state) {
	return getCalibrationSettings(library_state);
}

int UNITY_setCalibrationSettings(STATE_TYPE library_state, CalibrationSettings calibration_settings) {
	return setCalibrationSettings(library_state, calibration_settings);
}


