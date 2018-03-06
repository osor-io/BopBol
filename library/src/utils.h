#pragma once

#include <opencv2\opencv.hpp>
#include <vector>

//
//	This is a portable way to have a "press key to continue" function defined
//	in PRESS_TO_CONTINUE. As far as I know this should work on windows (first branch)
//	and MacOS/Linux (second branch)
//
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#define PRESS_TO_CONTINUE system("pause");
#else
#define PRESS_TO_CONTINUE system("read");
#endif

#define LOG(x) cout << x << endl;

//
//	Resizes the given image so it was the given width
//	but maintaining the aspect ratio
//
void utilscv_resize(cv::Mat * image, int width);


//
//	Resizes the image dividing only by powers of 2
//	and maintaining the aspect ratio
//
void utilscv_resizeCloseTo(cv::Mat * image, int width);

//
//	Sorting functionality for the quadrilateral contour
//
void utilscv_sortSquarePoints(std::vector<cv::Point2f> * data);

