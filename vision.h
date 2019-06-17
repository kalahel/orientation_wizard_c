//
// Created by GUEDJOU on 2019-06-13.
//

#ifndef SMARTDRONEGIT_VISION_H
#define SMARTDRONEGIT_VISION_H

#include "opencv2/opencv.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstring>

#define OPT 7
#define DEBUG 0
#define INFINITY 9999
#define ALPHA 0.01

using namespace cv;
using namespace std;
using namespace chrono;

/***** Structures *****/

typedef struct {
    double x, y;
} Coordinates;

typedef unsigned char byte;

typedef struct img {
    int rows;
    int cols;
    byte **data;
} image;


/*** Fonctions ****/
milliseconds getTime();
image img_allocation(int rows, int cols);
void Mat2byte_copy(Mat *image, struct img *out);
void get_interest_zone(Mat *image, Rect2d *roi, struct img *out);
void update_imagette(struct img *image_intereset, struct img *new_roi);
void img_distances(Mat *frame, struct img *image_interest, Rect2d *roi);
Coordinates total_correlation(struct img *frame, struct img *image_target);
void update_roi(Mat *frame, struct img *image_interest, Rect2d *roi);
void update_roi(Mat *frame, struct img *image_interest, Rect2d *roi);
void rotateMatrix(Mat *frame, Mat *outPut, Rect2d *roi, double teta);
void rotateRoi(Rect2d *roi, double teta);
Mat read_image(String file);
Coordinates get_ROI(Mat *frame, struct img *image_target);
int track_target(String file);
int track_target_video(String file);

#endif //SMARTDRONEGIT_VISION_H