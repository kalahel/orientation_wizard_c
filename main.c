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

Environment environment;
PointGPS position;
VectorGPS vectGPSGlobal;
VectorPOLAR vectPolarGlobal;
double distanceToDestination;

PointGPS createPoint(double latitude, double longitude) {
    PointGPS point;
    point.latitude = latitude;
    point.longitude = longitude;
    return point;
}

PointPOLAR createPointPOLAR(double angle, double radius) {
    PointPOLAR point;
    point.angle = angle;
    point.radius = radius;
    return point;
}

VectorGPS createVectorGPS(double d_long, double d_lat) {
    VectorGPS vectorGPS;
    vectorGPS.d_long = d_long;
    vectorGPS.d_lat = d_lat;
    return vectorGPS;
}

VectorPOLAR createVectorPOLAR(double d_angle, double d_radius) {
    VectorPOLAR vectorPolar;
    vectorPolar.d_angle = d_angle;
    vectorPolar.d_radius = d_radius;
    return vectorPolar;
}

VectorGPS createGpsVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB) {
    return createVectorGPS(pointB.longitude - pointA.longitude, pointB.latitude - pointA.latitude);
}

VectorGPS sumOfGpsVectorsAB(VectorGPS vectorA, VectorGPS vectorB) {
    return createVectorGPS(vectorA.d_long + vectorB.d_long, vectorA.d_lat + vectorB.d_lat);
}

double rad_2d_deg(double radians) {
    return radians * (180 / M_PI);
}

double deg_2_rad(double deg) {
    return deg * (M_PI / 180);
}

VectorPOLAR createPolarVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB) {
    VectorGPS vectorGps = createGpsVectorFromPointGpsAPointGpsB(pointA, pointB);
    double radius = sqrt(vectorGps.d_lat * vectorGps.d_lat + vectorGps.d_long * vectorGps.d_long);
    double angle = 0.0;
    if (vectorGps.d_long >= 0) {
        if (vectorGps.d_lat >= 0) {
            // atan2 returns radians. To convert it in degrees we do the following :
            angle = 90 - ((180.0 / M_PI) * (atan2(vectorGps.d_long, vectorGps.d_lat)));
        } else {
            angle = (180.0 / M_PI) * (atan2(-vectorGps.d_long, vectorGps.d_lat)) + 90;
        }
    } else {
        if (vectorGps.d_lat >= 0) {
            angle = (180.0 / M_PI) * (atan2(vectorGps.d_long, -vectorGps.d_lat)) + 270;
        } else {
            angle = (180.0 / M_PI) * (atan2(-vectorGps.d_long, -vectorGps.d_lat)) + 180;
        }
    }
    return createVectorPOLAR(angle, radius);
}

VectorPOLAR createPolarVectorFromPointGpsAPointGpsBv2(PointGPS pointA, PointGPS pointB) {
    VectorGPS vectorGps = createGpsVectorFromPointGpsAPointGpsB(pointA, pointB);
    double angle = 2*atan(vectorGps.d_lat/(vectorGps.d_long+sqrt(pow(vectorGps.d_long, 2) + pow(vectorGps.d_lat, 2))));
}

VectorPOLAR convertGpsVectorInPolarVector(VectorGPS vectorGps) {
    double radius = sqrt(vectorGps.d_lat * vectorGps.d_lat + vectorGps.d_long * vectorGps.d_long);
    double angle = 0.0;
    if (vectorGps.d_long >= 0) {
        if (vectorGps.d_lat >= 0) {
            // atan2 returns radians. To convert it in degrees we do the following :
            angle = 90-((180.0 / M_PI) * (atan2(vectorGps.d_long, vectorGps.d_lat)));
        } else {
            angle = (180.0 / M_PI) * (atan2(-vectorGps.d_long, vectorGps.d_lat)) + 90;
        }
    } else {
        if (vectorGps.d_lat >= 0) {
            angle = (180.0 / M_PI) * (atan2(vectorGps.d_long, -vectorGps.d_lat)) + 270;
        } else {
            angle = (180.0 / M_PI) * (atan2(-vectorGps.d_long, -vectorGps.d_lat)) + 180;
        }
    }
    return createVectorPOLAR(angle, radius);
}

VectorGPS convertPolarVectorInGpsVector(VectorPOLAR vectorPolar) {
    double longitude = vectorPolar.d_radius * cos(vectorPolar.d_angle * (M_PI / 180));
    double latitude = vectorPolar.d_radius * sin(vectorPolar.d_angle * (M_PI / 180));
    return createVectorGPS(longitude, latitude);
}

Obstacle createObstacle(int id, double radius, PointGPS position) {
    Obstacle obstacle;
    obstacle.id = id;
    obstacle.radius = radius;
    obstacle.position = position;
    return obstacle;
}

Environment createEnvironment(PointGPS destination, Obstacle *obstacles, int number_of_obstacles) {
    Environment environment;
    environment.destination = destination;
    environment.obstacles = obstacles;
    environment.obstacles_counter = number_of_obstacles;
    return environment;
}

void printObstacle(Obstacle *obstacle) {
    printf("Id: %d, Position: (%f, %f), Radius: %f\n", obstacle->id, obstacle->position.longitude,
           obstacle->position.latitude, obstacle->radius);
}

void printVectorPolar(VectorPOLAR vectorPolar){
    printf("Vector polar - radius : %lf , angle %lf\r\n", vectorPolar.d_radius, vectorPolar.d_angle);
}

void printVectorGPS(VectorGPS vectorGps){
    printf("Vector GPS - x : %lf , y : %lf\r\n", vectorGps.d_lat, vectorGps.d_long);
}

void printEnvironment(Environment environment) {
    printf("Destination: (%f, %f)\nList of obstacles:\n", environment.destination.longitude,
           environment.destination.latitude);
    if(environment.obstacles_counter > 0){
        int i = 0;
        for(i; i<environment.obstacles_counter; i++){
            printObstacle(&environment.obstacles[i]);
        }
    }else printf("There are no abstacles registered\r\n");
}

double computeOrthodormicDistance(PointGPS pointGps1, PointGPS pointGps2) {

    double earthRadius = 6378137;
    double rlat1 = deg_2_rad(pointGps1.latitude);
    double rlat2 = deg_2_rad(pointGps2.latitude);
    double rlong1 = deg_2_rad(pointGps1.longitude);
    double rlong2 = deg_2_rad(pointGps2.longitude);

    double dlong = (rlong2 - rlong1) / 2;
    double dlat = ((rlat2 - rlat1) / 2);

    double sub_calc = (sin(dlat) * sin(dlat) + cos(rlat1) * cos(rlat2) * sin(dlong) * sin(dlong));

    double d = 2 * atan2(sqrt(sub_calc), sqrt(1 - sub_calc));

    return (earthRadius * d);

    /*return  1852 * (60*acos(
            sin(deg_2_rad(pointGps1.latitude))
            *sin(deg_2_rad(pointGps2.latitude))
            +
            cos(deg_2_rad(pointGps1.latitude))
            *cos(deg_2_rad(pointGps2.latitude))
            *cos(deg_2_rad(pointGps2.longitude) - deg_2_rad(pointGps1.longitude))));*/
}

//FIXME : works like crap
double computeCarthesianDistance(PointGPS pointGps1, PointGPS pointGps2) {
    return sqrt(pow(pointGps2.latitude - pointGps1.latitude, 2) +
                pow((pointGps2.latitude - pointGps1.latitude) * (cos(pointGps2.latitude + pointGps1.latitude) / 2), 2));
}

double getDistanceBetweenPoints(PointGPS pointGps1, PointGPS pointGps2) {
    return computeOrthodormicDistance(pointGps1, pointGps2);
}

VectorGPS computeRepulsiveForceFromObstacle(Obstacle obstacle, VectorPOLAR attraction_vector) {
    printf("Compute repulsive force from obstacle :\r\n");
    double distance = computeOrthodormicDistance(obstacle.position, position);

    printf("      Distance : %lf", distance);

    VectorPOLAR repulsionVector = createPolarVectorFromPointGpsAPointGpsB(obstacle.position, position);

    printf("      repulsion vector from obstacle : ");
    if (distance < obstacle.radius + 15) {
        repulsionVector.d_radius = -2 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 30) {
        repulsionVector.d_radius = -1.5 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 50) {
        repulsionVector.d_radius = -attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 70) {
        repulsionVector.d_radius = -0.8 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 100) {
        repulsionVector.d_radius = -0.6 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 150) {
        repulsionVector.d_radius = -0.4 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 250) {
        repulsionVector.d_radius = -0.2 * attraction_vector.d_radius;
    } else if (distance < obstacle.radius + 500) {
        repulsionVector.d_radius = -0.1 * attraction_vector.d_radius;
    } else repulsionVector.d_radius = 0;
    repulsionVector.d_radius = - repulsionVector.d_radius;
    printf("      radius : %lf, angle : %lf\r\n", repulsionVector.d_radius, repulsionVector.d_angle);
    return convertPolarVectorInGpsVector(repulsionVector);
}

VectorGPS computeAttractiveForceFromDestination(){
    printf("Compute attractive force from destination :\r\n");
    distanceToDestination = computeOrthodormicDistance(position, environment.destination);
    printf("      Distance : %lf", distanceToDestination);
    VectorPOLAR attractionVector = createPolarVectorFromPointGpsAPointGpsB(position, environment.destination);
    printf("      attraction vector from destination : ");
    if (distanceToDestination < 10) {
        attractionVector.d_radius = 0;
    } else if (distanceToDestination < 30) {
        attractionVector.d_radius = 0.1;
    } else if (distanceToDestination <  50) {
        attractionVector.d_radius = 0.2;
    } else if (distanceToDestination < 70) {
        attractionVector.d_radius = 0.3;
    } else if (distanceToDestination < 100) {
        attractionVector.d_radius = 0.4;
    } else if (distanceToDestination < 150) {
        attractionVector.d_radius = 0.5;
    } else if (distanceToDestination < 250) {
        attractionVector.d_radius = 0.6;
    } else if (distanceToDestination < 500) {
        attractionVector.d_radius = 0.8;
    } else attractionVector.d_radius = 1;
    //attractionVector.d_radius = - attractionVector.d_radius;
    printf("      radius : %lf, angle : %lf\r\n", attractionVector.d_radius, attractionVector.d_angle);
    return convertPolarVectorInGpsVector(attractionVector);
}

VectorGPS computeDriverVectorFromEnvironement() {
    printf("Computing driver vector from the environement\r\n");
    printf("Vecteur d'attraction :\r\n");
    int i;
    //VectorGPS attractionVectorGPS = createGpsVectorFromPointGpsAPointGpsB(position, environment.destination);
    VectorGPS attractionVectorGPS = computeAttractiveForceFromDestination();
    printVectorGPS(attractionVectorGPS);
    VectorPOLAR attractionVectorPOLAR = convertGpsVectorInPolarVector(attractionVectorGPS);
    printVectorPolar(attractionVectorPOLAR);

    printf("Vecteur global : \r\n");
    VectorGPS globalVector = attractionVectorGPS;
    printVectorGPS(globalVector);
    for (i = 0; i < environment.obstacles_counter; i++) {
        globalVector = sumOfGpsVectorsAB(globalVector, computeRepulsiveForceFromObstacle(environment.obstacles[i],
                                                                                         attractionVectorPOLAR));
        printf("Vecteur global :\r\n");
        printVectorGPS(globalVector);
    }
    printVectorPolar(convertGpsVectorInPolarVector(globalVector));
    return globalVector;
}

VectorGPS computeVectorGPSFromDestination() {
    return createGpsVectorFromPointGpsAPointGpsB(position, environment.destination);
}

VectorPOLAR computeVectorPOLARFromDestination() {
    return convertGpsVectorInPolarVector(createGpsVectorFromPointGpsAPointGpsB(position, environment.destination));
}

void collectCurrentPosition(){
    position.longitude = (atof(gps_data.longitude)/100)+0.073080;//-0.123585
    position.latitude = (atof(gps_data.latitude)/100)+0.369184;//+0.52485 +0.369184
}

void run_test_with_no_obstacles(){
    init_serial_read();
    PointGPS destination = createPoint(48.923209, 2.182694);
    Obstacle obstacles[0];
    environment = createEnvironment(destination, obstacles, 0);
    FILE* log_file;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        get_gps_data();
        print_gps_data();
        collectCurrentPosition();
        //if (gps_data.longitude[0] != 'M' && gps_data.longitude[1] != '(') {
        if (gps_data.longitude[0] != 'M') {
            //printf("\r\n\r\nAS TEXT \r\n Latitude : %s\r\n Longitude : %s\r\n",gps_data.latitude,gps_data.longitude);

            //printf("AS DOUBLE \r\n Latitude : %lf\r\n Longitude : %lf\r\n\r\n\r\n",position.latitude,position.longitude);
            vectGPSGlobal = computeDriverVectorFromEnvironement();
            vectPolarGlobal = convertGpsVectorInPolarVector(vectGPSGlobal);

            printVectorPolar(vectPolarGlobal);

            //printf("\r\n vect carth = Latitude : %lf , Longitude : %lf", vectGPSGlobal.d_lat, vectGPSGlobal.d_long);
            log_file = fopen("./log_without_obstacles.txt", "a");
            /*
             * FORMAT OF THE OUTPUT IS :
             * position.lat, position.long, destination.lat, destination.long, distance, vectPolar.radius, vectPolar.angle, vectGPS.lat, vectGPS.long
             */
            fprintf(log_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n",
                    position.latitude,
                    position.longitude,
                    destination.latitude,
                    destination.longitude,
                    distanceToDestination,
                    vectPolarGlobal.d_angle,
                    vectPolarGlobal.d_radius,
                    vectGPSGlobal.d_lat,
                    vectGPSGlobal.d_long
            );
            fclose(log_file);
            if(distanceToDestination <15){
                //EXEC YASMINE
            }
        }else printf("No data yet");

    }
#pragma clang diagnostic pop
}

void run_test_with_obstacles(){
    init_serial_read();
    PointGPS destination = createPoint(48.923209, 2.182694);
    PointGPS pointGps1 = createPoint(48.787972, 1.584565);
    PointGPS pointGps2 = createPoint(49.202650, 6.925839);
    Obstacle obstacle1 = createObstacle(1, 10, pointGps1);
    Obstacle obstacle2 = createObstacle(2, 30, pointGps2);
    Obstacle obstacles[2];
    obstacles[0] = obstacle1;
    obstacles[1] = obstacle2;
    environment = createEnvironment(destination, obstacles, 2);
    FILE* log_file;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        get_gps_data();
        print_gps_data();
        collectCurrentPosition();
        if (gps_data.longitude[0] != 'M') {
            vectGPSGlobal = computeDriverVectorFromEnvironement();
            vectPolarGlobal = convertGpsVectorInPolarVector(vectGPSGlobal);
            printVectorPolar(vectPolarGlobal);
            log_file = fopen("./log_with_obstacles.txt", "a");
            /*
             * FORMAT OF THE OUTPUT IS :
             * position.lat, position.long, destination.lat, destination.long, distance, vectPolar.radius, vectPolar.angle, vectGPS.lat, vectGPS.long
             */
            fprintf(log_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n",
                    position.latitude,
                    position.longitude,
                    destination.latitude,
                    destination.longitude,
                    distanceToDestination,
                    vectPolarGlobal.d_angle,
                    vectPolarGlobal.d_radius,
                    vectGPSGlobal.d_lat,
                    vectGPSGlobal.d_long
            );
            fclose(log_file);
            if(distanceToDestination <15){
                //EXEC YASMINE
            }
        }else printf("No data yet");

    }
#pragma clang diagnostic pop
}


int main() {
    //run_test_with_no_obstacles();
/*    PointGPS pointGps1 = createPoint(48.787972, 1.584565);
    PointGPS pointGps2 = createPoint(49.202650, 6.925839);
    PointGPS destination = createPoint(48.721869, 1.491401);
    Obstacle obstacle1 = createObstacle(1, 10, pointGps1);
    Obstacle obstacle2 = createObstacle(2, 30, pointGps2);
    Obstacle obstacles[2];
    obstacles[0] = obstacle1;
    obstacles[1] = obstacle2;
    // environement is global
    environment = createEnvironment(destination, obstacles, 2);
    //printf("%d\n",environment.obstacles->id);
    printf("%d\n", environment.obstacles[1].id);
    printEnvironment(environment);
    printf("Distance : %lf\n", getDistanceBetweenPoints(pointGps1, destination));
    position = createPoint(48.752066, 1.565245);
    VectorGPS globalVect = computeDriverVectorFromEnvironement();

    PointGPS destination = createPoint(48.923927, 2.182639);
    init_serial_read();
    FILE* log_file;
    PointGPS p1 = createPoint(0,0);
    PointGPS p2 = createPoint(1,0.1);
    VectorPOLAR vectorPolar = createPolarVectorFromPointGpsAPointGpsB(p1,p2);
    printf("REsit : %lf %lf", vectorPolar.d_angle, vectorPolar.d_radius);
    log_file = fopen("./log.txt", "a");
    fprintf(log_file, "%lf %lf\r\n", vectorPolar.d_angle, vectorPolar.d_radius);
    fclose(log_file);

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        get_gps_data();
        print_gps_data();
        collectCurrentPosition();
        //if (gps_data.longitude[0] != 'M' && gps_data.longitude[1] != '(') {
        if (position.longitude != 0.0) {
            printf("\r\n\r\nAS TEXT \r\n Latitude : %s\r\n Longitude : %s\r\n",
                    gps_data.latitude,
                    gps_data.longitude);

            printf("AS DOUBLE \r\n Latitude : %lf\r\n Longitude : %lf\r\n\r\n\r\n",
                    position.latitude,
                    position.longitude);
            vectPolarGlobal = createPolarVectorFromPointGpsAPointGpsB(position, destination);
            vectGPSGlobal = createGpsVectorFromPointGpsAPointGpsB(position, destination);
            printf("\r\n vect polar = Radius : %lf , Angle : %lf", vectPolarGlobal.d_radius, vectPolarGlobal.d_angle);
            printf("\r\n vect carth = Latitude : %lf , Longitude : %lf", vectGPSGlobal.d_lat, vectGPSGlobal.d_long);
            log_file = fopen("./log.txt", "a");
            fprintf(log_file, "Position = %lf,%lf \r\n Destination = %lf,%lf \r\n VectPOL : %lf,%lf \r\n VectGPS ; %lf,%lf \r\n",
                              position.latitude,
                              position.longitude,
                              destination.latitude,
                              destination.longitude,
                              vectPolarGlobal.d_angle,
                              vectPolarGlobal.d_radius,
                              vectGPSGlobal.d_lat,
                              vectGPSGlobal.d_long
                              );
            fclose(log_file);
        }else printf("No data yet");

    }
#pragma clang diagnostic pop*/

    //PointGPS pointGps1 = createPoint(49.049446, 2.084837);
    //PointGPS pointGps2 = createPoint(49.050897, 2.081223);
    PointGPS destination = createPoint(48.923209, 2.182694);
    //Obstacle obstacle1 = createObstacle(1, 10, pointGps1);
    //Obstacle obstacle2 = createObstacle(2, 30, pointGps2);
    Obstacle obstacles[0];
    //obstacles[0] = obstacle1;
    //obstacles[1] = obstacle2;
    // environement is global
    environment = createEnvironment(destination, obstacles, 0);
    //printf("%d\n",environment.obstacles->id);
    printf("%d\n", environment.obstacles[0].id);
    printEnvironment(environment);
    position = createPoint(48.922665, 2.182725);

    printf("Distance between position and destination : %lf", computeOrthodormicDistance(position, destination));

    VectorGPS globalVect = computeDriverVectorFromEnvironement();
}
