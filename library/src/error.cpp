#include "error.h"
#include <iostream>

//
//	@@TODO: Send Unity the error using some kind of 
//	callback function.
//
void manageError(ERROR id, char* message) {

	if (message != NULL) {
		std::cerr << "ERROR [" << id << "]: " << message << std::endl;
	}
	else {
		std::cerr << "ERROR [" << id << "]" << std::endl;
	}
}

void manageError(ERROR_CALLBACK callback_function, ERROR id) {
	if (callback_function != NULL) {
		callback_function(id);
	}
}