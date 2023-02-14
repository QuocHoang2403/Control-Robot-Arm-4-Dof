#include <AccelStepper.h>
#include <MultiStepper.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <math.h>

#define PI 3.14159265359

#define dirPin_1 22
#define stepPin_1 24

#define dirPin_2 28
#define stepPin_2 30

#define dirPin_3 34
#define stepPin_3 36

AccelStepper stepper1(AccelStepper::DRIVER,stepPin_1,dirPin_1);
AccelStepper stepper2(AccelStepper::DRIVER,stepPin_2,dirPin_2);
AccelStepper stepper3(AccelStepper::DRIVER,stepPin_3,dirPin_3);
//-------------
float b1 = 0.198;
float a2 = 0.305;
float a3 = 0.3;
float d4 = 0.06;
bool setHome = false;
bool runDone = false;

typedef struct calc
{
    float theta1;
    float theta2;
    int n2;
    float theta3;
    int n3;
}theta;

float trajectory[8][3] = {{-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05},
                          {-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05},
                          {-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05},
                          {-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05}};
                          
float trajectory2[8][3] = {{-0.3,-0.1,0.05},
                          {-0.4,-0.1,0.05},
                          {-0.4,-0.3,0.05},
                          {-0.3,-0.3,0.05},
                          {-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05},
                          {-0.3,-0.1,0.05},
                          {-0.5,-0.1,0.05}};
float theta_list[8][3];
float theta_list2[8][3];
// float theta_list_test[3][3] = {{-45,90,-90},
//                                {-90,45,-45},
//                                {0,0,0}};
//-------------

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


void setPinMode(){
  DDRA |= (1<<DDA0)|(1<<DDA2) |(1<<DDA6);
  DDRC |= (1<<DDC1)|(1<<DDC3)|(1<<DDC7);
  DDRA &= ~(1<<DDA1)|(1<<DDA3)|(1<<DDA5);
  PORTA |= (1<<PORTA1)|(1<<PORTA3)|(1<<PORTA5);
}
void setParameter_1(float accel,double targetPosition){
  targetPosition = -targetPosition*4900/90;
  float speed = abs(targetPosition)/4;
  stepper1.setMaxSpeed(speed);
  stepper1.setSpeed(speed);
  stepper1.setAcceleration(accel);
  stepper1.moveTo(targetPosition);
}
void setParameter_2(float accel,double targetPosition){
  targetPosition = targetPosition*4750/90;
  float speed = abs(targetPosition)/4;
  stepper2.setMaxSpeed(speed);
  stepper2.setSpeed(speed);
  stepper2.setAcceleration(accel);
  stepper2.moveTo(targetPosition);
}
void setParameter_3(float accel,double targetPosition){
  targetPosition = -targetPosition*3550/90;
  float speed = abs(targetPosition)/4;
  stepper3.setMaxSpeed(speed);
  stepper3.setSpeed(speed);
  stepper3.setAcceleration(accel);
  stepper3.moveTo(targetPosition);
}
void move2Position(float n1,float n2,float n3){
  setParameter_1(6000,n1);
  setParameter_2(6000,n2);
  setParameter_3(6000,n3);
  while(stepper1.distanceToGo() != 0 ||stepper2.distanceToGo() != 0 ||stepper3.distanceToGo() != 0 )  
  {
    stepper1.run();
    stepper2.run();
    stepper3.run();

  }
}
void setup(){
  setPinMode();

  stepper1.setMaxSpeed(800);
  stepper1.setSpeed(800);
  stepper2.setMaxSpeed(800);
  stepper2.setSpeed(800);
  stepper3.setMaxSpeed(800);
  stepper3.setSpeed(800);
  
  theta ptr;
  for (int i = 0; i < 8; i++)
  {
    invKinematic(trajectory[i][0],trajectory[i][1],trajectory[i][2],PI/2,1,&ptr);
    theta_list[i][0] = ptr.theta1;

    theta_list[i][1] = ptr.theta2;

    theta_list[i][2] = ptr.theta3;
  }

  for (int i = 0; i < 8; i++)
  {
    invKinematic(trajectory2[i][0],trajectory2[i][1],trajectory2[i][2],PI/2,1,&ptr);
    theta_list2[i][0] = ptr.theta1;

    theta_list2[i][1] = ptr.theta2;

    theta_list2[i][2] = ptr.theta3;
  }
}

void loop() {
  int readHome = PINA & 0b00101010;
  if (setHome ==false){
    if(readHome == 42 ){
      stepper1.setPinsInverted(true);
      stepper1.runSpeed();
    }
    else if(readHome == 40 ){
      stepper1.setPinsInverted(false);
      stepper1.setCurrentPosition(0);
      stepper2.setPinsInverted(true);
      stepper2.runSpeed();
    }
    else if(readHome == 32 ){
      stepper2.setPinsInverted(false);
      stepper2.setCurrentPosition(0);
      stepper3.setPinsInverted(true);
      stepper3.runSpeed();
    }
    else if (readHome == 0 ){
      setHome = true;
      stepper3.setPinsInverted(false);
      stepper3.setCurrentPosition(0);
      }
  }
  if (setHome == true){
    for(int i = 0; i<7;i++){
      theta_list[i][0] = theta_list[i][0]*180/PI;
      theta_list[i][1] = theta_list[i][1]*180/PI;
      theta_list[i][2] = theta_list[i][2]*180/PI;
      move2Position(theta_list[i][0],theta_list[i][1],theta_list[i][2]);
      delay(1000);
    }
    for(int i = 0; i<5;i++){
      theta_list2[i][0] = theta_list2[i][0]*180/PI;
      theta_list2[i][1] = theta_list2[i][1]*180/PI;
      theta_list2[i][2] = theta_list2[i][2]*180/PI;
      move2Position(theta_list2[i][0],theta_list2[i][1],theta_list2[i][2]);
      delay(1000);
    }
    runDone = true;
  }
  // delay(1000);
  // move2Position(-90,90,0);
  while(runDone);
}
