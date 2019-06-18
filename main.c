#include "pathfinding.h"

int main() {
    run_test_yasmine();
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