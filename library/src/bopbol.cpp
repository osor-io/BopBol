#include "bopbol.h"
#include "libraryImplementation.h"

bool bbIsCallable() {
	return true;
}

int bbInit(STATE_TYPE library_state) {
	return initImageDetection(library_state);
}

int bbLaunch(STATE_TYPE library_state) {
	return launchImageProcessing(library_state);
}

int bbStop(STATE_TYPE library_state) {
	return stopImageDetection(library_state);
}

STATE_TYPE bbCreateInstance() {
	return createLibraryState();
}

void bbDestroyInstance(STATE_TYPE library_state) {
	return destroyLibraryState(library_state);
}

int bbSetBallHSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {
	return setBall_HSVRanges(library_state,
		iLowH, iHighH, iLowS,
		iHighS, iLowV, iHighV);
}

int bbSetBallRadiusThreshold(
	STATE_TYPE library_state,
	int radius) {
	return setBall_RadiusThreshold(library_state, radius);
}

int bbSetConfigurationParameters(
	STATE_TYPE library_state,
	int show_collisions,
	bool using_video_file,
	bool show_trackbars,
	bool output_frames) {
	return setConfigurationParameters(library_state,
		show_collisions, using_video_file,
		show_trackbars, output_frames);
}

int bbSetCoordinateCallback(
	STATE_TYPE library_state,
	COORDINATE_CALLBACK callback_function_ptr) {
	return setCoordinateCallback(library_state, callback_function_ptr);
}

int bbSetErrorCallback(
	STATE_TYPE library_state,
	ERROR_CALLBACK callback_function_ptr) {
	return setErrorCallback(library_state, callback_function_ptr);
}


int bbStartAreaCalibration(
	STATE_TYPE library_state) {
	return startProjectionCalibration(library_state);
}

ProjectionCalibration bbEndAreaCalibration(
	STATE_TYPE library_state) {
	return endProjectionCalibration(library_state);
}

int bbCalibrateAreaWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {
	return calibrateProjectionWithClick(library_state,
		hue_threshold,
		saturation_threshold,
		value_threshold);
}

int bbCalibrateAreaWithHSVRanges(
	STATE_TYPE library_state,
	int iLowH, int iHighH, int iLowS,
	int iHighS, int iLowV, int iHighV) {
	return calibrateProjectionWithHSVRanges(library_state,
		iLowH, iHighH, iLowS,
		iHighS, iLowV, iHighV);
}

IMAGE_DLL_API int bbCalibrateBallWithClick(
	STATE_TYPE library_state,
	int hue_threshold,
	int saturation_threshold,
	int value_threshold) {
	return calibrateBallWithClick(library_state,
		hue_threshold,
		saturation_threshold,
		value_threshold);
}


CalibrationSettings bbGetCalibrationSettings(STATE_TYPE library_state) {
	return getCalibrationSettings(library_state);
}

int bbSetCalibrationSettings(STATE_TYPE library_state, CalibrationSettings calibration_settings) {
	return setCalibrationSettings(library_state, calibration_settings);
}


