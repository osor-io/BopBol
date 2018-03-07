#pragma once

#include <cstring>
#include <iostream>
#include <opencv2/opencv.hpp>

#define DEQUE_LENGTH 100

struct Deque {

	//
	//	The data itself
	//
	cv::Point2f data[DEQUE_LENGTH];

	//
	//	The current position of the tail of the queue.
	//	By definition, the head of the queue is the previous
	//	element modulus the length
	//
	unsigned int tail_position;

	//
	//	Keeps track of the amount of elements currently
	//	inserted in the deque
	//
	unsigned int size;

};


//
//	Inits the deque and puts all the values to 0
//
void deque_init(Deque * deque);


//
//	Inserts an element in a given position
//
void deque_insertElement(Deque * deque, cv::Point2f element);


//
//	Returns the element located at a given position
//
cv::Point2f deque_getElementAt(Deque * deque, unsigned int position);




