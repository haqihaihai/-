#ifndef __mathematicalmodel_H
#define __mathematicalmodel_H	 

//float invSqrt(float number);

float mathematicalmodel_straight_xp(double angle1,double angle2,float l1,float l2,float e);
float mathematicalmodel_straight_yp(double  angle1,double angle2,float l1,float l2,float e);
float mathematicalmodel_contrary_corner1(float xp,float yp,float l1,float l2);
float mathematicalmodel_contrary_corner2(float xp,float yp,float l1,float l2);

float mathematicalmodelparallel_straight_xp(double angle1,double angle2,float l1,float l2,float e);
float mathematicalmodelparallel_straight_yp(double  angle1,double angle2,float l1,float l2,float e);
float mathematicalmodelparallel_contrary_corner1(float xp,float yp,float l1,float l2,float e);
float mathematicalmodelparallel_contrary_corner2(float xp,float yp,float l1,float l2,float e);
#define PI 3.14159265358979f
#define RADIAN(x) ((x)/180.0f*PI)

#endif
