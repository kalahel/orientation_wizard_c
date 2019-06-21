
#include "vision.h"
#include <unistd.h>

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
                //printf("Hello");
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
    //Mat2byte_copy(&img, &image_target);


    //Capture
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("Error\n");
        return -1;
    }

    Mat frame;
    cap >> frame;
    //Coordinates coor = get_ROI(&frame, &image_target);
    //Coordinates coor = detect_hist(&frame, &img);
    Coordinates coor = detect_hist_scaled(&frame, &img);
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


/**
 * Fonction qui me renvoie une matrice à partir d'une ROI
 *
 */

Mat get_matrix_roi ( Mat *frame, Rect *roi){

    Mat matrix_roi;
    //printf("Roi y %d Roi x %d Width %d  Height %d\n", roi->y, roi->x, roi->width, roi->height );
    return (*frame)( Range(roi->y, roi->height +roi->y - 1 ), Range( roi->x, roi->width +roi->x - 1 ) );

}

/**
 * Fonction qui renvoie le score entre deux histogrammes
 */

double get_score_histogramme(MatND *hist_1, MatND *hist_2) {

    double score =  compareHist( *hist_1, *hist_2, CV_COMP_BHATTACHARYYA);
    //printf("Score Fun %f\n", score);
    return score;

}


MatND generate_histograme(Mat *image) {

    Mat hsv_image, hsv_half_down;

    /// Convert to HSV
    cvtColor( (*image), hsv_image, COLOR_BGR2HSV );

    /// Using 50 bins for hue and 60 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };

    /// Histograms
    MatND hist_image;

    /// Calculate the histograms for the HSV images
    calcHist( &(*image), 1, channels, Mat(), hist_image, 2, histSize, ranges, true, false );
    normalize( hist_image, hist_image, 0, 1, NORM_MINMAX, -1, Mat() );

    return hist_image;
}

/**
 * Fonction qui calcule la corrélation totale et renvoie le vecteur de déplacement
*/
Coordinates detect_hist(Mat *frame, Mat *image_target) {

    double min;
    int rows, cols, rows_target, cols_target, roi_x, roi_y;
    Coordinates coor;
    Mat current_matrix;
    MatND current_hist;
    double current_score;

    //Initialisation
    min = 1.0;
    current_score=1;
    rows = (*frame).rows;
    cols = (*frame).cols;
    cols_target=(*image_target).rows;
    rows_target=(*image_target).rows;

    // calculer l'histograme de l'image target
    //printf("BEGIN - Target Histogram\n");
    MatND hist_target = generate_histograme(image_target);
    //printf("END - Target Histogram\n");

// Correlation
    for (int x = 0; x < cols; x += 5) {
        // printf(" X %d", x);
        for (int y = 0; y < rows; y += 5) {
            // printf("BEGIN-target frame\n");
            if ( (x+cols_target < cols)  && (y+rows_target < rows) ){
                //printf("BEGIN- x %d, cols %d, rows %d",x, cols_roi, rows_roi);
                Rect current_roi(x, y,cols_target, rows_target);
                current_matrix=get_matrix_roi (frame, &current_roi);
                current_hist=generate_histograme(&current_matrix);
                current_score=get_score_histogramme(&hist_target, &current_hist);
            }

            // printf("Min %f, Current Scotr %f, x %d, y %d\n", min, current_score, x, y);


            // Trouver le max
            if (current_score < min) {
                min =  current_score;
                coor.x = (double)x;
                coor.y = (double)y;
                // printf("Dans if  X %f, Y %f\n ", coor.x, coor.y);
            }
        }
    }
    //printf("Coordonnées envoyées : X %f , Y %f\n", coor.x, coor.y);

    return coor;
}

int track_target_hist(String file) {

    //get pixels of the taregt image
    Mat img = read_image(file);
    //struct img image_target = img_allocation(img.rows, img.cols);
    //Mat2byte_copy(&img, &image_target);


    //Capture
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("Error\n");
        return -1;
    }

    Mat frame;
    cap >> frame;
    //Coordinates coor = get_ROI(&frame, &image_target);
    Coordinates coor = detect_hist(&frame, &img);
    //get ROI
    Rect roi(coor.x, coor.y, img.cols, img.rows);

    //get histogram of target

    Mat roi_matrix=get_matrix_roi(&frame, &roi);
    MatND roi_hist=generate_histograme(&roi_matrix);





    //for (;;) {
    //cap >> frame;
    rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
    //show image with the tracked object
    imshow("Tracker", frame);
    if (waitKey(1) == 27) return 1;
    //}
    for (;;) {
        cap >> frame; //get a new frame from camera
        update_roi_hist(&frame, &roi, &roi_hist);
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
 * Fonction qui calcule ma ditance de bhattacharyya entre deux histograme sur les imagettes candidates et renvoie le maximum
 */

Coordinates get_distance_bhattacharyya (Mat *frame, Rect *roi, MatND *roi_hist) {

    int  rows, cols, cols_roi, rows_roi, roi_x, roi_y;
    Mat current_matrix;
    MatND current_hist;
    double min, current_score;
    Coordinates coor;

// get rows and cols
    rows = frame->rows;
    cols = frame->cols;
    cols_roi=roi->width;
    rows_roi=roi->height;
    roi_x=roi->x;
    roi_y=roi->y;


//Initialisation
    min = 1.0;
    current_score=1.0;

// Bhattacharyya distance X shifts
    for (int x = -OPT; x < OPT; x++) {

        //On parcourt l'imagette roi
        if ( (x+roi_x+cols_roi < cols) && (x+roi_x > 0) ){
            //printf("BEGIN- x %d, cols %d, rows %d",x, cols_roi, rows_roi);
            Rect current_roi(x+roi_x, roi_y,cols_roi, rows_roi);
            current_matrix=get_matrix_roi (frame, &current_roi);
            //printf("END--");
            current_hist=generate_histograme(&current_matrix);
            current_score=get_score_histogramme(roi_hist, &current_hist);

        }
        //printf("Minimum %f, Current Score %f, x %d\n", min, current_score, x);

        // Trouver le max
        if (current_score < min) {
            //printf(" Score %f  min %f \n", current_score, min);
            min =  current_score;
            coor.x = (double)x;
            //printf("Dans if x %f \n", coor.x );
        }
    }

    //Initialisation
    min = 1.0;

//   Bhattacharya distance on y shifts
    for (int y = -OPT; y < OPT; y++) {

        //On parcourt l'imagette roi
        if ( (y+roi_y+rows_roi< rows) && (y+roi_y>0)  ){
            // printf("BEGIN GET MATRIX ROI");
            Rect current_roi(roi_x, y+roi_y, cols_roi,rows_roi);
            current_matrix=get_matrix_roi (frame, &current_roi);
            // printf("END GET MATRIX ROI");


            current_hist=generate_histograme(&current_matrix);
            current_score=get_score_histogramme(roi_hist, &current_hist);
        }

        //printf("END-target frame\n");

        // Trouver le max
        if (current_score < min) {
            min =  current_score;
            coor.y = (double)y;
            //printf("Dans if y %f \n", coor.y );
        }
    }
    //printf("Coordonnées envoyées  x %f Coordonées y %f\n", coor.x, coor.y);
    return coor;


}


/**
 * Fonction qui met à jour le roi
 */
void update_roi_hist(Mat *frame, Rect *roi, MatND *roi_hist){

    Coordinates coor= get_distance_bhattacharyya (frame, roi, roi_hist);

    //printf("Coordonées Reçues X %f,  Y %f\n", coor.x, coor.y);

    //update roi
    (*roi) += Point(coor.x, coor.y);

}


/**
 * Fonction qui séélctionne le roi mannuellement
 */


/*int track_roi_manually(){

    //Capture
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        printf("Error\n");
        return -1;
    }

    Mat frame;
    cap >> frame;

    // select a bounding box
    Rect roi = selectROI(frame);

    //get histogram of
    // printf("BEGIN-- GET HIST ROI\n");
    Mat roi_matrix=get_matrix_roi(&frame, &roi);
    MatND roi_hist=generate_histograme(&roi_matrix);
    // printf("FIN-- GET HIST ROI");





//quit if ROI was not selected
    if (roi.width == 0 || roi.height == 0)
        return 0;


    for (;;) {

        cap >> frame;

        //update l'image reférente

        // get_interest_zone(&frame, &roi, &new_roi);
        //update_imagette (&image_interest, &new_roi);
        update_roi_hist(&frame, &roi, &roi_hist);

        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) break;
    }




}*/
/**
 * Fonction qui track sur une video
 *
 */

/*int track_roi_manually_video(String file){

    //Capture

    //VideoCapture video("video.MOV");
    VideoCapture video(file);
    //VideoCapture video("voiture-rouge.MOV");
    //VideoCapture video("occultation1-1.mov");

// Exit if video is not opened
    if (!video.isOpened()) {
        cerr << "Could not read video file" << endl;
        return 1;
    }

    Mat frame;
    bool ok = video.read(frame);
    if (!ok) {
        cerr << "Could not read the first frame" << endl;
        return 1;
    }

    // select a bounding box
    Rect roi = selectROI(frame);

//quit if ROI was not selected
    if (roi.width == 0 || roi.height == 0)
        return 0;

    //get histogram of
    // printf("BEGIN-- GET HIST ROI\n");
    Mat roi_matrix=get_matrix_roi(&frame, &roi);
    MatND roi_hist=generate_histograme(&roi_matrix);
    // printf("FIN-- GET HIST ROI");

    while (video.read(frame)) {

        //cout << "DEBUG 1 " << nb_frame <<  endl;
        //start tracking
        //update_roi(&frame, &image_interest, &roi);
        update_roi_hist(&frame, &roi, &roi_hist);

        //update l'image reférente
        // get_interest_zone(&frame, &roi, &new_roi);
        //update_imagette (&image_interest, &new_roi);

        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);

        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) break;
    }


}*/

/**
 * Fonction qui track manuellement à partir d'une video
 * @param frame
 * @param image_target
 * @return
 */
/* {

    int nb_frame=0;
    double fps;


//VIDEO
    VideoCapture video("miseajour-rotation3.mov");

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

//quit if ROI was not selected
   // if (roi.width == 0 || roi.height == 0)
   //     return 0;

    //selection for miseajour-rotation3
    Rect roi(79, 107, 155,56);
    //waitKey(0);


    //affichage des coordonnées du roi
    //cout << " Cordonnée" << "roix"<< roi.x << "roiy"<< roi.y << "roiheight"<< roi.height << "roi widht"<< roi.width << endl;

    // get interest zone
    struct img image_interest = img_allocation(roi.y + roi.height, roi.x + roi.width);
    struct img new_roi = img_allocation(roi.y + roi.height, roi.x + roi.width);
    get_interest_zone(&frame, &roi, &image_interest);

    cout << "Alpha  " << ALPHA << endl;

    while (video.read(frame)) {

        nb_frame++;

        //cout << "DEBUG 1 " << nb_frame <<  endl;
        //milliseconds start = getTime();
        //start tracking
        cout << "Frame number " << nb_frame << endl;
        milliseconds start = getTime();
        update_roi(&frame, &image_interest, &roi);




        //mettre à jour l'image cible
        get_interest_zone(&frame, &roi, &new_roi);
        update_imagette (&image_interest, &new_roi);

        fps=(getTime() - start).count();
        fps=1/fps*1000;
        cout  << "FPS " << fps <<  endl;


        // draw the tracked object
        rectangle(frame, roi, Scalar(255, 0, 0), 2, 1);
        //show image with the tracked object
        imshow("Tracker", frame);
        //quit on ESC button
        if (waitKey(1) == 27) break;
    }
    return 0;




}*/



/*
 *
 */
Coordinates detect_hist_scaled(Mat *frame, Mat *image_target) {

    double min;
    int rows, cols, rows_target, cols_target, scaled_rows, scaled_cols, scale_coeff;
    Coordinates coor;
    Mat current_matrix;
    MatND current_hist;
    double current_score;

    //Initialisation
    min = 1;
    rows = (*frame).rows;
    cols = (*frame).cols;
    cols_target = (*image_target).rows;
    rows_target = (*image_target).rows;


    // calculer l'histograme de l'image target
    printf("BEGIN - Target Histogram\n");
    MatND hist_target = generate_histograme(image_target);
    printf("END - Target Histogram\n");

    scale_coeff = 1;
    while (scale_coeff <= 30) {
        // Correlation
        for (int x = 0; x < cols; x += 5) {
            // printf(" X %d", x);
            for (int y = 0; y < rows; y += 5) {
                // printf("BEGIN-target frame\n");
                if ((x + ((int)cols_target*(scale_coeff*0.1)) < cols) && (y + ((int)rows_target*(scale_coeff*0.1) < rows))) {
                    Rect current_roi(x, y, x + ((int)cols_target*(scale_coeff*0.1)), y + ((int)rows_target*(scale_coeff*0.1)));
                    current_matrix = get_matrix_roi(frame, &current_roi);
                    current_hist = generate_histograme(&current_matrix);
                    current_score = get_score_histogramme(&hist_target, &current_hist);
                    //Display the current analysed zone
                        //update_roi(&frame, &image_target, &current_roi);
                    // draw rect
                    rectangle(*frame, current_roi, Scalar(255, 0, 0), 2, 0);
                    imshow("Tracker", *frame);
                    sleep(1);
                    if (current_score < min) {
                        min = current_score;
                        coor.x = x;
                        coor.y = y;
                    }
                }

                //printf("END-target frame\n");


                // Trouver le max

            }
        }
        scale_coeff += 2;
    }


    return coor;
}
