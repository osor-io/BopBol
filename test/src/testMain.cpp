#include <iostream>
#include <thread>
#include <chrono>
#include "bopbol.h"


#define TO_STREAM(stream, variable)                            \
  (stream) << #variable " (" << typeid(variable).name() << ")" \
           << ": " << (variable) << std::endl;
#define LOG(variable) TO_STREAM(std::cout, (variable))

#define MSG(msg)  std::cout << "Message Log: " << msg << std::endl << std::endl ;

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#define PRESS_TO_CONTINUE system("pause");
#else
#define PRESS_TO_CONTINUE system("read");
#endif

void processingThreadFunction(void* library_state) {
	bbLaunch(library_state);
}


int main() {

	MSG("Initializing testing project");

	MSG("Able to connect to DLL?");
	LOG(bbIsCallable());

	MSG("Creating library local state");
	void* library_state = bbCreateInstance();
	LOG(library_state);


	MSG("Testing configuration functions");
	bbSetBallHSVRanges(library_state, 23, 43, 30, 190, 50, 250);
	bbSetBallRadiusThreshold(library_state, 4);
	bbSetConfigurationParameters(library_state, 1, true, true, true);
	bbSetCoordinateCallback(library_state, [](float x, float y) -> int {
		LOG(x);
		LOG(y);
		MSG("\n\n");
		return 1;
	});
	bbSetErrorCallback(library_state, NULL);

	MSG("Initiating image detection");
	bbInit(library_state);

	MSG("Calibrating projection");
	bbStartAreaCalibration(library_state);
	MSG("Calibrating with click");
	bbCalibrateAreaWithClick(library_state, 10, 60, 60);
	ProjectionCalibration points = bbEndAreaCalibration(library_state);

	MSG("Calibrating ball with click");
	bbCalibrateBallWithClick(library_state, 10, 10, 10);

	MSG("Getting and setting calibration settings");
	CalibrationSettings settings = bbGetCalibrationSettings(library_state);
	bbSetCalibrationSettings(library_state, settings);

	MSG("Launching image processing");
	std::thread processing_thread(processingThreadFunction, library_state);

	MSG("Processing for 60 seconds");
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	MSG("Stopping image processing");
	bbStop(library_state);

	MSG("Waiting for the thread to join");
	processing_thread.join();


	MSG("SECOND TIME!");

	MSG("Launching image processing");
	std::thread processing_thread_2(processingThreadFunction, library_state);

	MSG("Processing for only 1 second");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	MSG("Stopping image processing");
	bbStop(library_state);

	MSG("Waiting for the thread to join");
	processing_thread_2.join();


	MSG("Destroying library state");
	bbDestroyInstance(library_state);

	MSG("We are more DONE than DANONE");
	PRESS_TO_CONTINUE;

	return 0;
}


