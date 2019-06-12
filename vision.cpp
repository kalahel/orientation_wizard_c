//
// Created by Henri on 12/06/2019.
//

#include "vision.h"

using namespace cv;
using namespace std;

/**** Global variables *****/
Coordinates coordinates;

/****** Functions ******/

/** Fonction qui alloue la mémoire **/
image img_allocation(int rows, int cols) {
    image out;
    out.rows = rows;
    out.cols = cols;

    out.data = (byte **) malloc(rows * sizeof(byte *));

    for (int i = 0; i < rows; i++)
        out.data[i] = (byte *) malloc(cols * sizeof(byte));

#if (DEBUG)
    printf("-FIN ALLOCATION- \n");
#endif
    return out;

}

/**
 * Fonction qui copie une frame dans la structure image
**/

void Mat2byte_copy(Mat *image, struct img *out) {

#if(DEBUG)
    printf("---COPY---- \n");
    printf("NB LIGNES :%d\n",image->rows);
    printf("NB COLS   :%d\n",image->cols);
#endif

    Vec3b color;
    for (int y = 0; y < image->rows; y++)
        for (int x = 0; x < image->cols; x++) {
            color = image->at<Vec3b>(Point(x, y));
            out->data[y][x] = (color.val[0] + color.val[1] + color.val[2]) / 3;
        }
#if (DEBUG)
    printf("-FIN COPY- \n");
#endif
}


/**
 *Fonction qui copie une frame dans la structure image
**/

void get_interest_zone(Mat *image, Rect2d *roi, struct img *out) {

//printf("---GET INTEREST ZONE---- \n");
//printf("NB LIGNES :%d\n",image->rows);
//printf("NB COLS   :%d\n",image->cols);

    //alloc mem
    //(*out) = img_allocation(roi.y + roi.height, roi.x + roi.width);
    Vec3b color;
    for (int y = roi->y; y < roi->y + roi->height; y++) {
        for (int x = roi->x; x < roi->x + roi->width; x++) {
            color = image->at<Vec3b>(Point(x, y));
            out->data[y - (int) roi->y][x - (int) roi->y] = (color.val[0] + color.val[1] + color.val[2]) / 3;
        }
    }
//printf("-FIN INTEREST ZONE - \n");
}


/**
 * Fonction qui calcule la distance entre deux imagettes
**/


void img_distances(Mat *frame, struct img *image_interest, Rect2d *roi) {

    int somme1, somme2, min, xMin, yMin, rows, cols;

// get rows and cols
    rows = frame->rows;
    cols = frame->cols;

//Allouer la mémoire pour la frame
    struct img image = img_allocation(rows, cols);

//Récupérer la data depuis la frame
    Mat2byte_copy(frame, &image);


//Initialisation
    xMin = yMin = 0;
    min = INFINITY;

// Correlation 1-D on X shifts
    for (int x = -OPT; x < OPT; x++) {
        //printf(" X opt %d\n",x);
        somme2 = 0;
        //On parcourt l'imagette roi
        for (int j = roi->y; j < roi->y + roi->height; j++) {
            somme1 = 0;
            for (int i = roi->x; i < roi->x + roi->width; i++) {
                if ((i + x < cols) && (i + x > 0) && (j < rows) && (j > 0)) {
                    somme1 = somme1 + ((int) image_interest->data[j - (int) roi->y][i - (int) roi->x] -
                                       (int) image.data[j][i + x]);
                }
            }
            somme2 = somme2 + somme1;

        }
        //Find x which minimises the distance
        if (somme2 < min) {
            min = somme2;
            xMin = x;
        }
    }

// get the x shit vector
    coordinates.x = xMin;


/**********   Correlation 1-D on y shifts   ********/
    for (int y = -OPT; y < OPT; y++) {
        somme2 = 0;
        //printf("1 x %d\n",x);
        //On parcourt l'imagette roi
        for (int j = roi->y; j < roi->y + roi->height; j++) {
            somme1 = 0;
            //printf("2 %d\n",i);
            for (int i = roi->x; i < roi->x + roi->width; i++) {

                if ((j + y < rows) && (j + y > 0) && (i < cols) && (i > 0)) {
                    //printf("Y %d, j %d , i %d roix %d roiy %d\n", y, j ,i, (int)roi.x, (int)roi.y);
                    somme1 = somme1 + ((int) image_interest->data[j - (int) roi->y][i - (int) roi->x] -
                                       (int) image.data[j + y][i]);
                }


            }
            somme2 += somme1;

        }
        //Find x which minimises the distance
        if (somme2 < min) {
            min = somme2;
            yMin = y;
        }
    }

//Get the y shift vector
    coordinates.y = yMin;

}

/**
 * Function witch updates bounding box coordinates
**/

void update_roi(Mat *frame, struct img *image_interest, Rect2d *roi) {

//calculate the distance between images
    img_distances(frame, image_interest, roi);

//get the vector shift
    (*roi) += Point2d(coordinates.x, coordinates.y);
}


int main(int argc, char **argv) {

// Read video
    //VideoCapture video("video.MOV");
    VideoCapture video("video3.mp4");

// Exit if video is not opened
    if (!video.isOpened()) {
        cerr << "Could not read video file" << endl;
        return 1;
    }

// Read first frame
    Mat frame;
    bool ok = video.read(frame);
    if (!ok) {
        cerr << "Could not read the first frame" << endl;
        return 1;
    }

// select a bounding box
    Rect2d roi;
    roi = selectROI("tracker", frame);

//quit if ROI was not selected
    if (roi.width == 0 || roi.height == 0)
        return 0;

// Display bounding box.
    rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
    imshow("Tracking", frame);

// get interest zone
    struct img image_interest = img_allocation(roi.y+roi.height, roi.x+roi.width);
    get_interest_zone(&frame, &roi, &image_interest);
    printf("Start Correlation\n");


    while (video.read(frame)) {
        update_roi(&frame, &image_interest, &roi);
        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) break;
    }
    return 0;
}