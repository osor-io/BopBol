#include "utils.h"
#include <cmath>
#include <list>
#include <cstdlib>
#include <iostream>


void utilscv_resize(cv::Mat * image, int width) {

	float resizing_ratio = (float)width / (float)image->size().width;
	int heigth_transformed = (int)std::floorf((float)image->size().height * (float)resizing_ratio);

	cv::resize(*image, *image, cv::Size(width, heigth_transformed));

}


void utilscv_resizeCloseTo(cv::Mat * image, int width) {

	int aux_width = image->size().width;

	int last_distance = INT_MAX;
	int power = 0;

	//
	//	Q: Is there a cooler way than this to do infinite loops?
	//	A: I don't think so mister...
	//
	for (;;) {

		int current_distance;

		//
		//	We divide by 2 the auxiliary resolution every iteration
		//	of the loop
		//
		aux_width /= 2;
		current_distance = abs(aux_width - image->size().width);

		//
		//	@@TODO: Maybe we should check here if we are going
		//	lower than the target width in case we jump into a really
		//	low resolution, that should not happen often since most
		//	original resolutions will be 720p, 480p or higher 
		//
		if (last_distance > current_distance) {
			last_distance = current_distance;

			//
			//	WE INCREASE OUR POWER
			//
			power++;
		}
		else {

			//
			//	When we finish we need to multiply the auxiliary resolution
			//	by 2 since we divide it earlier in the iteration
			//
			aux_width *= 2;
			break;
		}
	}

	int aux_height = (int)(image->size().height / (pow(2, power)));

	cv::resize(*image, *image, cv::Size(aux_width, aux_height));

}

void utilscv_sortSquarePoints(std::vector<cv::Point2f> * data) {

	if (data->size() != 4) {
		return;
	}

	//
	//	First we sort by rows
	//
	sort(data->begin(), data->end(), [](const cv::Point & a, const cv::Point & b) -> bool {
		return (a.y < b.y);
	});

	//
	//	And we swap the top two and the bottom two 
	//
	if ((*data)[0].x > (*data)[1].x) {
		cv::Point aux = (*data)[0];
		(*data)[0] = (*data)[1];
		(*data)[1] = aux;
	}

	if ((*data)[2].x < (*data)[3].x) {
		cv::Point aux = (*data)[2];
		(*data)[2] = (*data)[3];
		(*data)[3] = aux;
	}

	return;

}