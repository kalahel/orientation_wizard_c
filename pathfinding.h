//
// Created by Henri on 14/06/2019.
//
#ifndef ORIENTATION_WIZARD_C_PATHFINDING_H
#define ORIENTATION_WIZARD_C_PATHFINDING_H

#include <stdio.h>
#include <math.h>

#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <unistd.h>  /* UNIX Standard Definitions      */
#include <errno.h>   /* ERROR Number Definitions           */
#include <assert.h>
#include "serial_gps.h"

#define PI                  3.14159265359
#define NELEMS(x)           (sizeof(x) / sizeof((x)[0]))
#define DEG_2_RAD(x)        (x*PI)/180
#define ORTHODROMIC_DIST(lat1, long1, lat2, long2)    1852 * (60*acos(sin(DEG_2_RAD(lat1))*sin(DEG_2_RAD(lat2)) + cos(DEG_2_RAD(lat1))*cos(DEG_2_RAD(lat2))*cos(DEG_2_RAD(long2) - DEG_2_RAD(long1))))


/*
 * Structures
 */

// longitude <==> x
// latitude <==> y
struct PointGPS {
    double longitude;
    double latitude;
} typedef PointGPS;

struct PointPOLAR {
    double angle;
    double radius;
} typedef PointPOLAR;

struct VectorGPS {
    double d_long;
    double d_lat;
} typedef VectorGPS;

struct VectorPOLAR {
    double d_angle;
    double d_radius;
} typedef VectorPOLAR;

struct Obstacle {
    int id;
    PointGPS position;
    double radius;
} typedef Obstacle;

struct Environment {
    Obstacle *obstacles;
    PointGPS destination;
    int obstacles_counter;
} typedef Environment;


/*
 * Global variables
 */

Environment environment;
PointGPS position;
VectorGPS vectGPSGlobal;
VectorPOLAR vectPolarGlobal;
double distanceToDestination;

/*
 * Functions
 */

PointGPS createPoint(double latitude, double longitude);
PointPOLAR createPointPOLAR(double angle, double radius);
VectorGPS createVectorGPS(double d_long, double d_lat);
VectorPOLAR createVectorPOLAR(double d_angle, double d_radius);
VectorGPS createGpsVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB);
VectorGPS sumOfGpsVectorsAB(VectorGPS vectorA, VectorGPS vectorB);
double rad_2d_deg(double radians);
double deg_2_rad(double deg);
VectorPOLAR createPolarVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB);
VectorPOLAR createPolarVectorFromPointGpsAPointGpsBv2(PointGPS pointA, PointGPS pointB);
VectorPOLAR convertGpsVectorInPolarVector(VectorGPS vectorGps);
VectorGPS convertPolarVectorInGpsVector(VectorPOLAR vectorPolar);
Obstacle createObstacle(int id, double radius, PointGPS position);
Environment createEnvironment(PointGPS destination, Obstacle *obstacles, int number_of_obstacles);
void printObstacle(Obstacle *obstacle);
void printVectorPolar(VectorPOLAR vectorPolar);
void printVectorGPS(VectorGPS vectorGps);
void printEnvironment(Environment environment);
double computeOrthodormicDistance(PointGPS pointGps1, PointGPS pointGps2);
double computeCarthesianDistance(PointGPS pointGps1, PointGPS pointGps2);
double getDistanceBetweenPoints(PointGPS pointGps1, PointGPS pointGps2);
VectorGPS computeRepulsiveForceFromObstacle(Obstacle obstacle, VectorPOLAR attraction_vector);
VectorGPS computeAttractiveForceFromDestination();
VectorGPS computeDriverVectorFromEnvironement();
VectorGPS computeVectorGPSFromDestination();
VectorPOLAR computeVectorPOLARFromDestination();
void collectCurrentPosition();
void run_test_with_no_obstacles();
void run_test_with_obstacles();
void run_test_with_obstacles_static();
void run_test_yasmine();



#endif //ORIENTATION_WIZARD_C_PATHFINDING_H
