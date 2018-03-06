#include "bopbol.h"

#define SUCCESS	0;
#define FAILURE	-1;
enum ERROR {
	DOIN_GOOD,
	UNABLE_TO_OPEN_VIDEO,
	COULD_NOT_READ_FRAME,
	NOT_IN_CALIBRATION_MODE,
	COULD_NOT_CALIBRATE
};


void manageError(ERROR id, char* message = nullptr);

void manageError(ERROR_CALLBACK callback_function, ERROR id);