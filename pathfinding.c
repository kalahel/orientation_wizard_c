//
// Created by Henri on 14/06/2019.
//

#include "pathfinding.h"
#include "vision.h"

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
    double angle =
            2 * atan(vectorGps.d_lat / (vectorGps.d_long + sqrt(pow(vectorGps.d_long, 2) + pow(vectorGps.d_lat, 2))));
}

VectorPOLAR convertGpsVectorInPolarVector(VectorGPS vectorGps) {
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

void printVectorPolar(VectorPOLAR vectorPolar) {
    printf("Vector polar - radius : %lf , angle %lf\r\n", vectorPolar.d_radius, vectorPolar.d_angle);
}

void printVectorGPS(VectorGPS vectorGps) {
    printf("Vector GPS - x : %lf , y : %lf\r\n", vectorGps.d_lat, vectorGps.d_long);
}

void printEnvironment(Environment environment) {
    printf("Destination: (%f, %f)\nList of obstacles:\n", environment.destination.longitude,
           environment.destination.latitude);
    if (environment.obstacles_counter > 0) {
        int i = 0;
        for (i; i < environment.obstacles_counter; i++) {
            printObstacle(&environment.obstacles[i]);
        }
    } else printf("There are no abstacles registered\r\n");
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
    repulsionVector.d_radius = -repulsionVector.d_radius;
    printf("      radius : %lf, angle : %lf\r\n", repulsionVector.d_radius, repulsionVector.d_angle);
    return convertPolarVectorInGpsVector(repulsionVector);
}

VectorGPS computeAttractiveForceFromDestination() {
    printf("Compute attractive force from destination :\r\n");
    distanceToDestination = computeOrthodormicDistance(position, environment.destination);
    printf("      Distance : %lf", distanceToDestination);
    VectorPOLAR attractionVector = createPolarVectorFromPointGpsAPointGpsB(position, environment.destination);
    printf("      attraction vector from destination : ");
    if (distanceToDestination < 10) {
        attractionVector.d_radius = 0;
    } else if (distanceToDestination < 30) {
        attractionVector.d_radius = 50;
    } else if (distanceToDestination < 50) {
        attractionVector.d_radius = 100;
    } else if (distanceToDestination < 70) {
        attractionVector.d_radius = 200;
    } else if (distanceToDestination < 100) {
        attractionVector.d_radius = 300;
    } else if (distanceToDestination < 150) {
        attractionVector.d_radius = 500;
    } else if (distanceToDestination < 250) {
        attractionVector.d_radius = 700;
    } else if (distanceToDestination < 500) {
        attractionVector.d_radius = 1000;
    } else attractionVector.d_radius = 1414;
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

void collectCurrentPosition() {
    position.longitude = (atof(gps_data.longitude) / 100) + 0.073080;//-0.123585
    position.latitude = (atof(gps_data.latitude) / 100) + 0.369184;//+0.52485 +0.369184
}

void run_test_with_no_obstacles() {
    init_serial_read();
    PointGPS destination = createPoint(49.034201, 2.055462);
    Obstacle obstacles[0];
    environment = createEnvironment(destination, obstacles, 0);
    FILE *log_file;

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
            if (distanceToDestination < 15) {
                //EXEC YASMINE
            }
        } else printf("No data yet");

    }
#pragma clang diagnostic pop
}

void run_test_with_obstacles() {
    init_serial_read();
    PointGPS destination = createPoint(49.034121, 2.055532);
    PointGPS pointGps1 = createPoint(49.056166, 2.077470);
    PointGPS pointGps2 = createPoint(49.051642, 2.091098);
    PointGPS pointGps3 = createPoint(49.045828, 2.086153);
    Obstacle obstacle1 = createObstacle(1, 10, pointGps1);
    Obstacle obstacle2 = createObstacle(2, 30, pointGps2);
    Obstacle obstacle3 = createObstacle(3, 20, pointGps3);
    Obstacle obstacles[3];
    obstacles[0] = obstacle1;
    obstacles[1] = obstacle2;
    obstacles[2] = obstacle3;
    environment = createEnvironment(destination, obstacles, 3);
    FILE *log_file;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        get_gps_data();
        print_gps_data();
        collectCurrentPosition();
        if (gps_data.longitude[0] != 'M') {
            vectGPSGlobal = computeDriverVectorFromEnvironement();
            vectPolarGlobal = convertGpsVectorInPolarVector(vectGPSGlobal);
            //printVectorPolar(vectPolarGlobal);
            log_file = fopen("./log_with_obstacles.txt", "a");
            /*
             * FORMAT OF THE OUTPUT IS :
             * position.lat, position.long, destination.lat, destination.long, distance, vectPolar.radius, vectPolar.angle, vectGPS.lat, vectGPS.long
             */
            fprintf(log_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n",
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
            if (distanceToDestination < 15) {
                //EXEC YASMINE
            }
        } else printf("No data yet");

    }
#pragma clang diagnostic pop
}

void run_test_with_obstacles_static() {
    init_serial_read();
    PointGPS destination = createPoint(49.028438, 2.048090);
    PointGPS pointGps1 = createPoint(49.023119, 2.045946);
    PointGPS pointGps2 = createPoint(49.024072, 2.046704);
    PointGPS pointGps3 = createPoint(49.025997, 2.050769);
    Obstacle obstacle1 = createObstacle(1, 100, pointGps1);
    Obstacle obstacle2 = createObstacle(2, 100, pointGps2);
    Obstacle obstacle3 = createObstacle(3, 100, pointGps3);
    position = createPoint(49.020686, 2.047023);
    Obstacle obstacles[3];
    obstacles[0] = obstacle1;
    obstacles[1] = obstacle2;
    obstacles[2] = obstacle3;
    environment = createEnvironment(destination, obstacles, 3);
    FILE *log_file;

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        vectGPSGlobal = computeDriverVectorFromEnvironement();
        vectPolarGlobal = convertGpsVectorInPolarVector(vectGPSGlobal);
        printVectorPolar(vectPolarGlobal);
        log_file = fopen("./log_with_obstacles_static.txt", "a");
        /*
         * FORMAT OF THE OUTPUT IS :
         * position.lat, position.long, destination.lat, destination.long, distance, vectPolar.radius, vectPolar.angle, vectGPS.lat, vectGPS.long
         */
        fprintf(log_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n",
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
        printf("position.latitude :%lf\r\n position.longitude :%lf \r\n %lf \r\n %lf \r\n distance : %lf \r\n angle : %lf \r\n rayon : %lf \r\n %lf \r\n %lf\r\n",
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
        if (distanceToDestination < 15) {
            //EXEC YASMINE
        }
        int direction = 0;
        scanf("%d", &direction);
        if (direction == 1) {
            position.longitude += 0.000140;
        }
        else if (direction == 2) {
            position.latitude += 0.000090;
        }
        else if (direction == 3) {
            position.longitude -= 0.000140;
        }
        else if (direction == 4) {
            position.latitude -+ 0.000090;
        }
    }
#pragma clang diagnostic pop
}

void run_test_yasmine(){
    track_target("../vignette.png");
}