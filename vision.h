//
// Created by Henri on 12/06/2019.
//

#include "opencv2/opencv.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

#ifndef ORIENTATION_WIZARD_C_VISION_H
#define ORIENTATION_WIZARD_C_VISION_H

#define OPT 5
#define DEBUG 0
#define INFINITY 9999

typedef struct {
    double x, y;
} Coordinates;

typedef unsigned char byte;

typedef struct img {
    int rows;
    int cols;
    byte **data;
} image;

image img_allocation(int rows, int cols);
void Mat2byte_copy(Mat *image, struct img *out);
void get_interest_zone(Mat *image, Rect2d *roi, struct img *out);
void img_distances(Mat *frame, struct img *image_interest, Rect2d *roi);
void update_roi(Mat *frame, struct img *image_interest, Rect2d *roi);


#endif //ORIENTATION_WIZARD_C_VISION_H
