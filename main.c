#include <stdio.h>
#include <math.h>

#define PI                  3.14159265359
#define NELEMS(x)           (sizeof(x) / sizeof((x)[0]))
#define DEG_2_RAD(x)        (x*PI)/180
#define ORTHODROMIC_DIST(lat1, long1, lat2, long2)    1852 * (60*acos(sin(DEG_2_RAD(lat1))*sin(DEG_2_RAD(lat2)) + cos(DEG_2_RAD(lat1))*cos(DEG_2_RAD(lat2))*cos(DEG_2_RAD(long2) - DEG_2_RAD(long1))))

// longitude <==> x
// latitude <==> y
struct PointGPS {
    double longitude;
    double latitude;
}typedef PointGPS;

struct PointPOLAR {
    double angle;
    double radius;
}typedef PointPOLAR;

struct VectorGPS {
    double d_long;
    double d_lat;
}typedef VectorGPS;

struct VectorPOLAR{
    double d_angle;
    double d_radius;
}typedef VectorPOLAR;

struct Obstacle{
    int id;
    PointGPS position;
    double radius;
}typedef Obstacle;

struct Environment{
    Obstacle* obstacles;
    PointGPS destination;
}typedef Environment;

Environment environment;
PointGPS position;

PointGPS createPoint(double longitude, double latitude){
    PointGPS point;
    point.latitude = latitude;
    point.longitude = longitude;
    return point;
}

PointPOLAR createPointPOLAR(double angle, double radius){
    PointPOLAR point;
    point.angle = angle;
    point.radius = radius;
    return point;
}

VectorGPS createVectorGPS(double d_long, double d_lat){
    VectorGPS vectorGPS;
    vectorGPS.d_long = d_long;
    vectorGPS.d_lat = d_lat;
    return vectorGPS;
}

VectorPOLAR createVectorPOLAR(double d_angle, double d_radius){
    VectorPOLAR vectorPolar;
    vectorPolar.d_angle = d_angle;
    vectorPolar.d_radius = d_radius;
    return vectorPolar;
}

VectorGPS createGpsVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB){
    return createVectorGPS(pointB.longitude - pointA.longitude, pointB.latitude - pointA.latitude);
}

VectorGPS sumOfGpsVectorsAB(VectorGPS vectorA, VectorGPS vectorB){
    return createVectorGPS(vectorA.d_long + vectorB.d_long, vectorA.d_lat + vectorB.d_lat);
}

double rad_2d_deg(double radians){
    return radians * ( 180 / M_PI);
}

double deg_2_rad(double deg){
    return deg * ( M_PI / 180);
}

VectorPOLAR createPolarVectorFromPointGpsAPointGpsB(PointGPS pointA, PointGPS pointB){
    VectorGPS vectorGps = createGpsVectorFromPointGpsAPointGpsB(pointA, pointB);
    double radius = sqrt(vectorGps.d_lat * vectorGps.d_lat + vectorGps.d_long * vectorGps.d_long);
    double angle = 0.0;
    if(vectorGps.d_long >=0){
        if(vectorGps.d_lat >=0){
            // atan2 returns radians. To convert it in degrees we do the following :
            angle = (180.0 / M_PI) *(atan2(vectorGps.d_long, vectorGps.d_lat));
        }else{
            angle = (180.0 / M_PI) *(atan2(-vectorGps.d_long, vectorGps.d_lat)) + 90;
        }
    }else{
        if(vectorGps.d_lat >=0){
            angle = (180.0 / M_PI) *(atan2(vectorGps.d_long, -vectorGps.d_lat))+270;
        }else{
            angle = (180.0 / M_PI) *(atan2(-vectorGps.d_long, -vectorGps.d_lat)) + 180;
        }
    }
    return createVectorPOLAR(angle, radius);
}

VectorPOLAR convertGpsVectorInPolarVector(VectorGPS vectorGps){
    double radius = sqrt(vectorGps.d_lat * vectorGps.d_lat + vectorGps.d_long * vectorGps.d_long);
    double angle = 0.0;
    if(vectorGps.d_long >=0){
        if(vectorGps.d_lat >=0){
            // atan2 returns radians. To convert it in degrees we do the following :
            angle = (180.0 / M_PI) *(atan2(vectorGps.d_long, vectorGps.d_lat));
        }else{
            angle = (180.0 / M_PI) *(atan2(-vectorGps.d_long, vectorGps.d_lat)) + 90;
        }
    }else{
        if(vectorGps.d_lat >=0){
            angle = (180.0 / M_PI) *(atan2(vectorGps.d_long, -vectorGps.d_lat))+270;
        }else{
            angle = (180.0 / M_PI) *(atan2(-vectorGps.d_long, -vectorGps.d_lat)) + 180;
        }
    }
    return createVectorPOLAR(angle, radius);
}

VectorGPS convertPolarVectorInGpsVector(VectorPOLAR vectorPolar){
    double longitude = vectorPolar.d_radius*    cos(vectorPolar.d_angle*(M_PI/180));
    double latitude = vectorPolar.d_radius*     sin(vectorPolar.d_angle*(M_PI/180));
    return createVectorGPS(longitude, latitude);
}

Obstacle createObstacle(int id, double radius, PointGPS position){
    Obstacle obstacle;
    obstacle.id = id;
    obstacle.radius = radius;
    obstacle.position = position;
    return obstacle;
}

Environment createEnvironment(PointGPS destination, Obstacle* obstacles){
    Environment environment;
    environment.destination = destination;
    environment.obstacles = obstacles;
    return environment;
}

void printObstacle(Obstacle *obstacle){
    printf("Id: %d, Position: (%f, %f), Radius: %f\n", obstacle->id, obstacle->position.longitude, obstacle->position.latitude, obstacle->radius);
}

void printEnvironment(Environment environment){
    printf("Destination: (%f, %f)\nList of obstacles:\n", environment.destination.longitude, environment.destination.latitude);
    printObstacle(&environment.obstacles[0]);
    printObstacle(&environment.obstacles[1]);
}

//FIXME : error with the distance computation
double computeOrthodormicDistance( PointGPS pointGps1, PointGPS pointGps2){

    double earthRadius = 6378137;
    double rlat1 = deg_2_rad(pointGps1.latitude);
    double rlat2 = deg_2_rad(pointGps2.latitude);
    double rlong1 = deg_2_rad(pointGps1.longitude);
    double rlong2 = deg_2_rad(pointGps2.longitude);

    double dlong = (rlong2 - rlong1) / 2;
    double dlat = ((rlat2 - rlat1) / 3);

    double sub_calc = ( sin(dlat) * sin(dlat) + cos(rlat1) * cos(rlat2) * sin(dlong) * sin(dlong));

    double d = 2 * atan2(sqrt(sub_calc), sqrt(1-sub_calc));

    return (earthRadius * d);

    /*return  1852 * (60*acos(
            sin(deg_2_rad(pointGps1.latitude))
            *sin(deg_2_rad(pointGps2.latitude))
            +
            cos(deg_2_rad(pointGps1.latitude))
            *cos(deg_2_rad(pointGps2.latitude))
            *cos(deg_2_rad(pointGps2.longitude) - deg_2_rad(pointGps1.longitude))));*/
}

double computeCarthesianDistance( PointGPS pointGps1, PointGPS pointGps2){
    return sqrt(pow(pointGps2.latitude - pointGps1.latitude, 2) + pow((pointGps2.latitude - pointGps1.latitude)*(cos(pointGps2.latitude + pointGps1.latitude)/2), 2));
}

double getDistanceBetweenPoints(PointGPS pointGps1, PointGPS pointGps2){
    return computeOrthodormicDistance(pointGps1, pointGps2);
}

VectorGPS computeRepulsiveForceFromObstacle(Obstacle obstacle, VectorPOLAR attraction_vector){
    double distance = ORTHODROMIC_DIST(obstacle.position.latitude, obstacle.position.longitude, environment.destination.latitude, environment.destination.longitude);
    VectorPOLAR repulsionVector = createPolarVectorFromPointGpsAPointGpsB(obstacle.position, position);
    if( distance < obstacle.radius + 15 ){
        repulsionVector.d_radius = -2 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 30 ){
        repulsionVector.d_radius = -1.5 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 50 ){
        repulsionVector.d_radius = -attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 70 ){
        repulsionVector.d_radius = -0.8 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 100 ){
        repulsionVector.d_radius = -0.6 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 150 ){
        repulsionVector.d_radius = -0.4 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 250 ){
        repulsionVector.d_radius = -0.2 * attraction_vector.d_radius;
    }else if( distance < obstacle.radius + 500 ) {
        repulsionVector.d_radius = -0.1 * attraction_vector.d_radius;
    }else repulsionVector.d_radius = 0;
    return convertPolarVectorInGpsVector(repulsionVector);
}

VectorGPS computeDriverVectorFromEnvironement(){
    int i;
    VectorGPS attractionVectorGPS = createGpsVectorFromPointGpsAPointGpsB(position, environment.destination);
    VectorPOLAR attractionVectorPOLAR = convertGpsVectorInPolarVector(attractionVectorGPS);
    VectorGPS globalVector = attractionVectorGPS;
    for(i = 0; i<NELEMS(environment.obstacles); i++){
        globalVector = sumOfGpsVectorsAB(globalVector, computeRepulsiveForceFromObstacle(environment.obstacles[i], attractionVectorPOLAR));
    }
    return globalVector;
}

// Convertir de dégrés en radians
int main() {
    PointGPS pointGps1 = createPoint(48.787972, 1.584565);
    PointGPS pointGps2 = createPoint(49.202650, 6.925839);
    PointGPS destination = createPoint(48.721869, 1.491401);
    Obstacle obstacle1 = createObstacle(1, 10, pointGps1 );
    Obstacle obstacle2 = createObstacle(2, 30, pointGps2 );
    Obstacle obstacles[2];
    obstacles[0] = obstacle1;
    obstacles[1] = obstacle2;
    // environement is global
    environment = createEnvironment(destination, obstacles );
    //printf("%d\n",environment.obstacles->id);
    printf("%d\n",environment.obstacles[1].id);
    printEnvironment(environment);
    printf("Distance : %lf\n",getDistanceBetweenPoints(pointGps1, destination));
    VectorGPS globalVect = computeDriverVectorFromEnvironement();
}