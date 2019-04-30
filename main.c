#include <stdio.h>

struct PointGPS {
    double longitude;
    double latitude;
}typedef PointGPS;

struct PointPOLAR {
    double angle;
    double value;
}typedef PointPOLAR;

struct VectorGPS {
    double d_long;
    double d_lat;
}typedef VectorGPS;

struct VectorPOLAR{
    double d_angle;
    double d_value;
}typedef VectorPOLAR;

PointGPS createPoint(double longitude, double latitude){
    PointGPS point;
    point.latitude = latitude;
    point.longitude = longitude;
    return point;
}

PointPOLAR createPointPOLAR(double angle, double value){
    PointPOLAR point;
    point.angle = angle;
    point.value = value;
    return point;
}

VectorGPS createVectorGPS(double d_long, double d_lat){
    VectorGPS vectorGPS;
    vectorGPS.d_long = d_long;
    vectorGPS.d_lat = d_lat;
    return vectorGPS;
}

VectorPOLAR createVectorPOLAR(double d_angle, double d_value){
    VectorPOLAR vectorPolar;
    vectorPolar.d_angle = d_angle;
    vectorPolar.d_value = d_value;
    return vectorPolar;
}


int main() {

}