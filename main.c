#include <stdio.h>
#include <math.h>

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

VectorGPS gpsVectorPointAPointB(PointGPS pointA, PointGPS pointB){
    return createVectorGPS(pointB.longitude - pointA.longitude, pointB.latitude - pointA.latitude);
}

VectorGPS addGPSVectorAB(VectorGPS vectorA, VectorGPS vectorB){
    return createVectorGPS(vectorA.d_long + vectorB.d_long, vectorA.d_lat + vectorB.d_lat);
}

VectorPOLAR pointGPSA_pointGPSB_toPolarVector(PointGPS pointA, PointGPS pointB){
    VectorGPS vectorGps = gpsVectorPointAPointB(pointA, pointB);
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
            angle = (180.0 / M_PI) *(atan2(vectorGps.d_long, -vectorGps.d_lat));
        }else{
            angle = (180.0 / M_PI) *(atan2(-vectorGps.d_long, -vectorGps.d_lat)) + 90;
        }
    }
}


int main() {

}