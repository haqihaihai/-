//#include "force_control.h"
//#include "foot_trajectory.h"
//#include "mathematicalmadel.h"
//#include <math.h>

//extern _pos1 poselec;
//_pos2 poslast;
//_matrix1 matrixnow;
//_distance distance1;
//int i=0,j=0,k=0;

//void force_control(float kx,float ky,float bx,float by,float l1,float l2,float t2,float Tmx,float T3)
//{
//	//gait_f(t2);
//	matrixnow.matrix_1[0][0]=l1*cos(distance1.M1)+l2*cos(distance1.M1+distance1.M2);
//	matrixnow.matrix_1[0][1]=l1*sin(distance1.M1)+l2*sin(distance1.M1+distance1.M2);
//	matrixnow.matrix_1[1][0]=l2*cos(distance1.M1+distance1.M2);
//	matrixnow.matrix_1[1][1]=l2*sin(distance1.M1+distance1.M2);
//	if(t2>=Tmx&&t2<T3)
//	{
//		matrixnow.matrix_2[0][0]=bx*(poselec.x1-poslast.x1_last);
//		matrixnow.matrix_2[1][0]=ky*(poselec.y1-poslast.y1_last)+by*(poselec.y1-poslast.y1_last);
//	}
//	else
//	{
//		matrixnow.matrix_2[0][0]=kx*(poselec.x1-poslast.x1_last)+bx*(0-poslast.x1_last);
//		matrixnow.matrix_2[1][0]=ky*(poselec.y1-poslast.y1_last)+by*(0-poslast.y1_last);
//	}
//	for (int i = 0; i < 2; i++)
//	{
//        for (int j = 0; j < 2; j++) 
//		{
//			matrixnow.matrix_3[i][0]+=matrixnow.matrix_1[i][j]*matrixnow.matrix_2[j][0];
//		}
//    }
//	poslast.x1_last=mathematicalmodel_straight_xp(distance1.M1,distance1.M2,l1,l2,2);
//	poslast.y1_last=mathematicalmodel_straight_yp(distance1.M1,distance1.M2,l1,l2,2);
//	distance1.M1=matrixnow.matrix_3[0][0];
//	distance1.M2=matrixnow.matrix_3[1][0];
//}

