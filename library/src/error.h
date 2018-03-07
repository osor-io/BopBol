#include "bopbol.h"


enum BbError {
	DOIN_GOOD,
	UNABLE_TO_OPEN_VIDEO,
	COULD_NOT_READ_FRAME,
	NOT_IN_CALIBRATION_MODE,
	COULD_NOT_CALIBRATE
};


void manageError(BbError id, char* message = nullptr);

void manageError(BB_ERROR_CALLBACK callback_function, BbError id);