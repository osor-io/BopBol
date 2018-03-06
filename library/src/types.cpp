#include "types.h"

//
//	Comments explaining the types and functions are
//	in types.h as you probably could have guessed :)
//

using namespace cv;

void deque_init(Deque * deque) {

	//
	//	Initially we set all the variables to 0
	//	It works with floats which is the thing that
	//	Point2f has inside, so no problem here
	//
	std::memset(&(deque->data), 0, sizeof(Point2f)*DEQUE_LENGTH);

	//
	//	And position and size start and 0
	//	which is pretty self explanatory right?
	//
	deque->tail_position = 0;
	deque->size = 0;

}

void deque_insertElement(Deque * deque, Point2f element) {

	deque->data[deque->tail_position] = element;
	deque->tail_position = (deque->tail_position + 1) % DEQUE_LENGTH;
	deque->size = (deque->size + 1 > DEQUE_LENGTH - 1 ? DEQUE_LENGTH - 1 : deque->size + 1);

}

Point2f deque_getElementAt(Deque * deque, unsigned int position) {

	int offset_position = (deque->tail_position - 1 - position);

	if (offset_position < 0) {
		return deque->data[DEQUE_LENGTH + offset_position];
	}
	else {
		return deque->data[offset_position % DEQUE_LENGTH];
	}

}
