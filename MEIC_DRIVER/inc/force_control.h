#ifndef __force_control_H
#define __force_control_H

typedef struct _position2{
	float x1_last;                 
	float y1_last;                 
	float x2_last;                 
	float y2_last;						
	float x3_last;	
	float y3_last;                 
	float x4_last;                 
	float y4_last;
}_pos2;

typedef struct matrix{
	float matrix_1[2][2];
	float matrix_2[2][1];
	float matrix_3[2][1];
}_matrix1;

typedef struct force_distance{
	float M1;
	float M2;
}_distance;

void force_control(float kx,float ky,float bx,float by,float l1,float l2,float t2,float Tmx,float T3);

#endif

