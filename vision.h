//
// Created by GUEDJOU on 2019-06-13.
//
#include "opencv2/opencv.hpp"
//#include <opencv2/gpu/device/utility.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdio.h>
#include <cstring>

/*
 *
 */

#ifndef SMARTDRONEGIT_VISION_H
#define SMARTDRONEGIT_VISION_H

#define OPT 7
#define DEBUG 0
#define INFINITY 9999
#define ALPHA 0.01

using namespace cv;
using namespace std;
//using namespace chrono;

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
image img_allocation(int rows, int cols);
void Mat2byte_copy(Mat *image, struct img *out);
void get_interest_zone(Mat *image, Rect *roi, struct img *out);
void update_imagette(struct img *image_intereset, struct img *new_roi);
void img_distances(Mat *frame, struct img *image_interest, Rect *roi);
Coordinates total_correlation(struct img *frame, struct img *image_target);
void update_roi(Mat *frame, struct img *image_interest, Rect *roi);
void update_roi(Mat *frame, struct img *image_interest, Rect *roi);
void rotateMatrix(Mat *frame, Mat *outPut, Rect *roi, double teta);
void rotateRoi(Rect *roi, double teta);
Mat read_image(String file);
Coordinates get_ROI(Mat *frame, struct img *image_target);
int track_target(String file);
int track_target_video(String file);
Mat get_matrix_roi ( Mat *frame, Rect *roi);
double get_score_histogramme(MatND *hist_1, MatND *hist_2)
MatND generate_histograme (Mat image);
Coordinates detect_hist(Mat *frame, Mat *image_target);
Coordinates detect_hist_scaled(Mat *frame, Mat *image_target);
int track_target_hist(String file);
void update_roi_hist(Mat *frame, Rect *roi, MatND *roi_hist)


#endif //SMARTDRONEGIT_VISION_H
