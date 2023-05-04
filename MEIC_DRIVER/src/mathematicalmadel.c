#include "mathematicalmadel.h"
#include <math.h>

//l1为大腿，l2为小腿（串联）

/*函数名：invSqrt(void)
	描述：求平方根的倒数
	该函数是经典的Carmack求平方根算法，效率极高，使用魔数0x5f375a86
*/

float invSqrt(float number) 
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );
    return y;
}

float mathematicalmodel_straight_xp(double  angle1,double  angle2,float l1,float l2,float e)//横坐标
{
	float xp;
	xp=(-1)*l1*cos(RADIAN(angle1))+e-l2*cos(RADIAN(angle2));
	return xp;
}

float mathematicalmodel_straight_yp(double  angle1,double angle2,float l1,float l2,float e)//纵坐标
{
	float yp;
	yp=(l1*sin(RADIAN(angle1))+l2*sin(RADIAN(angle2)))*(-1);
	return yp;
}

float mathematicalmodel_contrary_corner1(float xp,float yp,float l1,float l2)//转角1 大腿
{
	float angle;
	double angle1cos,angle2cos;
	angle1cos=(-xp)/sqrt((xp)*(xp)+yp*yp);
	angle2cos=(xp*xp+yp*yp+l1*l1-l2*l2)/(2*l1*sqrt((xp)*(xp)+yp*yp));
	angle=acos(angle1cos)+acos(angle2cos);
	angle=angle*180/PI;	
	return angle;
}

float mathematicalmodel_contrary_corner2(float xp,float yp,float l1,float l2)//转角2，小腿
{
	float angle;
	double angle1sin;
	angle1sin=((-1)*l1*cos(RADIAN(mathematicalmodel_contrary_corner1(xp,yp,l1,l2)))-xp)/l2;
	angle=acos(angle1sin);
	angle=angle*180/PI;
	return angle;
}

//以下为并联

float mathematicalmodelparallel_straight_xp(double  angle1,double  angle2,float l1,float l2,float e)//横坐标
{
	float xp,h,xd,d;
	xd=(l1*cos(RADIAN(angle1))+l1*cos(RADIAN(angle2)))/2;	
	d=sqrt((l1*cos(RADIAN(angle1))-l1*cos(RADIAN(angle2))+2*e)*(l1*cos(RADIAN(angle1))-l1*cos(RADIAN(angle2))+2*e)+(l1*sin(RADIAN(angle1))-l1*sin(RADIAN(angle2)))*(l1*sin(RADIAN(angle1))-l1*sin(RADIAN(angle2))));
	h=sqrt(l2*l2-d*d/4);
    xp=xd-h*((l1*sin(RADIAN(angle1))-l1*sin(RADIAN(angle2)))/d);
	return xp;
}

float mathematicalmodelparallel_straight_yp(double  angle1,double  angle2,float l1,float l2,float e)//纵坐标
{
	float yp,h,yd,d;
	yd=(l1*sin(RADIAN(angle1))+l1*sin(RADIAN(angle2)))/2;	
	d=sqrt((l1*cos(RADIAN(angle1))-l1*cos(RADIAN(angle2))+2*e)*(l1*cos(RADIAN(angle1))-l1*cos(RADIAN(angle2))+2*e)+(l1*sin(RADIAN(angle1))-l1*sin(RADIAN(angle2)))*(l1*sin(RADIAN(angle1))-l1*sin(RADIAN(angle2))));
	h=sqrt(l2*l2-d*d/4);
    yp=yd+h*((l1*cos(RADIAN(angle1))-l1*cos(RADIAN(angle2))+2*e)/d);
	return yp;
}

float mathematicalmodelparallel_contrary_corner1(float xp,float yp,float l1,float l2,float e)
{
	float angle;
	double angle1cos,angle2cos;
	angle1cos=(xp-e)/sqrt((xp-e)*(xp-e)+yp*yp);
	angle2cos=((xp-e)*(xp-e)+yp*yp+l1*l1-l2*l2)/(2*l1*sqrt((xp-e)*(xp-e)+yp*yp));
	angle=acos(angle1cos)-acos(angle2cos);
	angle=angle*180/PI;
	return angle;
}

float mathematicalmodelparallel_contrary_corner2(float xp,float yp,float l1,float l2,float e)
{
	float angle;
	double angle1cos,angle2cos;
	angle1cos=(xp+e)/sqrt((xp+e)*(xp+e)+yp*yp);
	angle2cos=((xp+e)*(xp+e)+yp*yp+l1*l1-l2*l2)/(2*l1*sqrt((xp+e)*(xp+e)+yp*yp));
	angle=acos(angle1cos)+acos(angle2cos);
	angle=angle*180/PI;
	return angle;
}

//void ni_jie(double Xp,double Yp,double l1,double l2,double e)//输入足端坐标 得到电机角度angleA angleB
//{
//	double a,b,c,d,f,g,ql,q2,q3,q4;
//	a=Xp-e;
//	b=Xp+e;
//	c=1/invSqrt(a*a+Yp*Yp);
//	d=a*a+Yp*Yp+l1*l1-l2*l2;
//	f=1/invSqrt(b*b+Yp*Yp);
//	g=b*b+Yp*Yp+l1*l1-l2*l2;
//	
//	ql=acos (a/c);
//	q2=acos( d/(2*l1*c));
//	q3=acos(b/f);
//	q4=acos( g/(2*l1*f));
//	
//	angleA=RadAngle(g1-g2);
//	angleB=RadAngle (g3+q4);
//	
//	if(angleA>=-71 && angleA<=95) angleA=angleA+45;// -26 -> 140 (289-360)
//	else
//	{
//		angleA_error=1;
//		printf("angleA逆解Error");
//	}
//	if(angleB>=88 && angleB<=250) angleB=225-angleB;// -25 -> 137
//	else
//	{
//		angleB_error=1;
//		printf("angleB逆解Error")；
//	}
//}

