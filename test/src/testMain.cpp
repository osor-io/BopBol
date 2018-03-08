#include <iostream>
#include <thread>
#include <chrono>
#include "bopbol.h"


#define TO_STREAM(stream, variable)                            \
  (stream) << #variable " (" << typeid(variable).name() << ")" \
           << ": " << (variable) << std::endl
#define LOG(variable) TO_STREAM(std::cout, (variable))

#define MSG(msg)  std::cout << "Message Log: " << msg << std::endl << std::endl

#define ENDL  std::cout << std::endl

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#define PRESS_TO_CONTINUE system("pause");
#else
#define PRESS_TO_CONTINUE system("read");
#endif

void processingThreadFunction(BbInstance instance) {
	bbLaunch(instance);
}


int main() {

	MSG("Initializing testing project");

	MSG("Library Version:");
	MSG(BB_VERSION);


	MSG("Able to connect to DLL?");
	LOG(bbIsCallable());

	MSG("Creating library local state");
	BbInstance instance = bbCreateInstance();
	LOG(instance);


	MSG("Testing configuration functions");
	bbSetBallHSVRanges(instance, 23, 43, 30, 190, 50, 250);
	bbSetBallRadiusThreshold(instance, 4);
	bbSetConfigurationParameters(instance, 1, true, true, true);
	bbSetCoordinateCallback(instance, [](float x, float y) -> int {
		LOG(x);
		LOG(y);
		ENDL;
		ENDL;
		return 1;
	});
	bbSetErrorCallback(instance, NULL);

	MSG("Initiating image detection");
	bbInit(instance);

	MSG("Calibrating projection");
	bbStartAreaCalibration(instance);
	MSG("Calibrating with click");
	bbCalibrateAreaWithClick(instance, 10, 60, 60);
	BbAreaCalibration points = bbEndAreaCalibration(instance);

	MSG("Calibrating ball with click");
	bbCalibrateBallWithClick(instance, 10, 10, 10);

	MSG("Getting and setting calibration settings");
	BbCalibrationSettings settings = bbGetCalibrationSettings(instance);
	bbSetCalibrationSettings(instance, settings);

	MSG("Launching image processing");
	std::thread processing_thread(processingThreadFunction, instance);

	MSG("Processing for 60 seconds");
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));

	MSG("Stopping image processing");
	bbStop(instance);

	MSG("Waiting for the thread to join");
	processing_thread.join();


	MSG("SECOND TIME!");

	MSG("Launching image processing");
	std::thread processing_thread_2(processingThreadFunction, instance);

	MSG("Processing for only 1 second");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	MSG("Stopping image processing");
	bbStop(instance);

	MSG("Waiting for the thread to join");
	processing_thread_2.join();


	MSG("Destroying library state");
	bbDestroyInstance(instance);

	MSG("We are more DONE than DANONE");
	PRESS_TO_CONTINUE;

	return 0;
}


