#include <stdio.h>
#include <math.h>

#define PI 3.14159265359

float b1 = 0.198;
float a2 = 0.305;
float a3 = 0.3;
float d4 = 0.06;
float trajectory[5][3] = {{0.131,-0.131,0.25},
                          {0.131,-0.4,0.25},
                          {0.4,-0.4,0.25},
                          {0.4,-0.131,0.25},
                          {0.605,0,0.198}};
float theta_list[5][3];
typedef struct calc
{
    float theta1;
    float theta2;
    float theta3;
}theta;

void invKinematic(float Px, float Py, float Pz,float theta4,int mode,theta *ptr){
    //calculate theta 1//
    float alpha = atan2(d4,(a2+a3));
    ptr->theta1 = atan2(Py,Px)-alpha;
    //calculate theta 3//
    float const1 = Px*cos(ptr->theta1) + Py*sin(ptr->theta1);
    float const2 = Pz - b1;
    float Cosx = (pow(const1,2) + pow(const2,2) - pow(a3,2) - pow(a2,2))/(2*a2*a3);
    float theta3_Temp[2];
    theta3_Temp[0] = acos(Cosx);
    theta3_Temp[1] = -acos(Cosx);

    // calculate theta 2//

    float r = sqrt(pow(Px,2)+pow(Py,2));
    float D = sqrt(pow((Pz-b1),2)+pow(r,2));
    float theta2_Temp[2];
    theta2_Temp[0] = atan2((Pz-b1),r) + asin((sin(theta3_Temp[0])*a3)/D);
    theta2_Temp[1] = atan2((Pz-b1),r) + asin((sin(theta3_Temp[1])*a3)/D);
    if (mode>0){
        ptr->theta2 = theta2_Temp[0];
        ptr->theta3 = theta3_Temp[1];
    }
    else if (mode<0){
        ptr->theta2 = theta2_Temp[1];
        ptr->theta3 = theta3_Temp[0];
    }
    

}   

int main()
{
    theta ptr;
    for (int i = 0; i < 4; i++)
    {
        invKinematic(trajectory[i][0],trajectory[i][1],trajectory[i][2],PI/2,1,&ptr);
        theta_list[i][0] = ptr.theta1;
        theta_list[i][1] = ptr.theta2;
        theta_list[i][2] = ptr.theta3;

        theta_list[i][0] = theta_list[i][0]*180/PI;
        theta_list[i][1] = theta_list[i][1]*180/PI;
        theta_list[i][2] = theta_list[i][2]*180/PI;
        printf("theta_list[%d][%f]\n",i,theta_list[i][0]);
        printf("theta_list[%d][%f]\n",i,theta_list[i][1]);
        printf("theta_list[%d][%f]\n",i,theta_list[i][2]);
        printf("            \n\n");
        
    }
    
    
    return 0;
}