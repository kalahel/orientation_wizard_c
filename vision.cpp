
#include "vision.h"
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
 * Fonction qui copie une matrice dans la structure image
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

void get_interest_zone(Mat *image, Rect *roi, struct img *out) {
#if (DEBUG)
    printf("---GET INTEREST ZONE---- \n");
    pintf("NB LIGNES :%d\n",image->rows);
    printf("NB COLS   :%d\n",image->cols);
#endif
    Vec3b color;
    for (int y = roi->y; y < roi->y + roi->height; y++) {
        for (int x = roi->x; x < roi->x + roi->width; x++) {
            color = image->at<Vec3b>(Point(x, y));
            out->data[y - (int) roi->y][x - (int) roi->y] = (color.val[0] + color.val[1] + color.val[2]) / 3;
        }
    }
#if (DEBUG)
    printf("-FIN INTEREST ZONE - \n");
#endif

}

/*
 * Fonction qui met à jour l'imagette
 */
void update_imagette(struct img *image_intereset, struct img *new_roi) {

    for (int y = 0; y < image_intereset->rows; y++) {
        for (int x = 0; x < image_intereset->cols; x++) {
            image_intereset->data[y][x] = (1 - ALPHA) * (image_intereset->data[y][x]) + (ALPHA * new_roi->data[y][x]);
        }
    }
}


/**
 * Fonction qui calcule la distance entre deux imagettes
**/


void img_distances(Mat *frame, struct img *image_interest, Rect *roi) {

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

//   Correlation 1-D on y shifts
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
 * Fonction qui calcule la corrélation totale et renvoie le vecteur de déplacement
*/
Coordinates total_correlation(struct img *frame, struct img *image_target) {

    int somme1, somme2, max, rows, cols, rows_it, cols_it;
    Coordinates coor;
//Initialisation
    max = -INFINITY;
    rows = (*frame).rows;
    cols = (*frame).cols;
    rows_it = (*image_target).rows;
    cols_it = (*image_target).cols;


// Correlation
    for (int x = 0; x < cols; x += 5) {
        // printf(" X %d", x);
        for (int y = 0; y < rows; y += 5) {
            somme2 = 0;
            for (int j = 0; j < rows_it; j++) {
                somme1 = 0;
                for (int i = 0; i < cols_it; i++)
                    if ((i + x < cols) && (j + y < rows))
                        somme1 += (int) image_target->data[j][i] * (int) frame->data[j + y][i + x];
                somme2 += somme1;
            }
            // Trouver le max
            if (somme2 > max) {
                max = somme2;
                coor.x = x;
                coor.y = y;
            }
        }
    }


    return coor;
}


/**
 * Function witch updates bounding box coordinates
**/

void update_roi(Mat *frame, struct img *image_interest, Rect *roi) {

//calculate the distance between images
    img_distances(frame, image_interest, roi);

//get the vector shift
    (*roi) += Point(coordinates.x, coordinates.y);
}

/**
 * Fonction qui fait la rotation entre deux images en fonction de la valeur de l'angle du nord
 */

void rotateMatrix(Mat *frame, Mat *outPut, Rect *roi, double teta) {

    double x, y, angle;
    Vec3b color;

    //récupérer les pixels
    struct img image = img_allocation(frame->rows, frame->cols);
    Mat2byte_copy(frame, &image);

    //définir l'angle à 90°
    angle = (teta * M_PI) / 180.0;

    for (int j = roi->y; j < roi->y + roi->height; j++) {
        for (int i = roi->x; i < roi->x + roi->width; i++) {

            if (teta == 90.0) {
                printf("Hello");
                x = j;
                y = -i;
            } else {
                x = (i * cos(angle)) - (j * sin(angle));
                y = (j * cos(angle)) + (i * sin(angle));
            }
            color.val[0] = image.data[j][i];
            color.val[1] = image.data[j][i];
            color.val[2] = image.data[j][i];
            outPut->at<Vec3b>(Point(x, y)) = color;
            //outPut->at<Vec3b>(Point(i,j))=0;
        }
    }
}

/**
 * Fonction qui fait la rotation de roi
 */

void rotateRoi(Rect *roi, double teta) {

    double angle;

    //définir l'angle à 90°
    angle = (teta * M_PI) / 180.0;
    double x, y;
    x = roi->x;
    y = roi->y;
    if (angle == 90.0) {
        roi->x = -y;
        roi->y = x;
    } else {
        roi->x = (x * cos(angle)) - (y * sin(angle));
        roi->y = (y * cos(angle)) + (x * sin(angle));
    }

}

/**
 * Fonction qui lie une image et vérifie si elle est valide
 */

Mat read_image(String file) {

    //lire l'image
    Mat img = imread(file);
    if (!img.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
    }
    return img;
}


/**
 * Fonction qui détecte l'emplacement d'une cible
 */
Coordinates get_ROI(Mat *frame, struct img *image_target) {

    struct img image_frame = img_allocation(frame->rows, frame->cols);
    Mat2byte_copy(frame, &image_frame);

    //faire la correlation totale
    Coordinates coor = total_correlation(&image_frame, image_target);

    return coor;
}

/**
 * Fonction qui réalise le tracking
 *
 */
int track_target(String file) {

    //get pixels of the taregt image
    Mat img = read_image(file);
    struct img image_target = img_allocation(img.rows, img.cols);
    Mat2byte_copy(&img, &image_target);


    //Capture
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("Error\n");
        return -1;
    }

    Mat frame;
    cap >> frame;
    Coordinates coor = get_ROI(&frame, &image_target);
    //get ROI
    Rect roi(coor.x, coor.y, img.cols, img.rows);

    //for (;;) {
        //cap >> frame;
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        if (waitKey(1) == 27) return 1;
   //}
    for (;;) {
        cap >> frame; //get a new frame from camera
        update_roi(&frame, &image_target, &roi);
        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) return 1;

    }

    return 0;
}

/**
 * Fonction qui réalise le tracking
 *
 */
int track_target_video(String file) {

    //get pixels of the taregt image
    Mat img = read_image(file);
    struct img image_target = img_allocation(img.rows, img.cols);
    Mat2byte_copy(&img, &image_target);


//VIDEO
    VideoCapture video("occultation2.MOV");
    //VideoCapture video("voiture-rouge.MOV");
    //VideoCapture video("occultation2.MOV");

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

    //Detect the ROI
    Coordinates coor = get_ROI(&frame, &image_target);
    Rect roi(coor.x, coor.y, img.cols, img.rows);

    for (;;) {
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) break;

    }
    return 0;
}
