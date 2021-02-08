#include <math.h>
//#include< cstdio>
//k#include ".. \include\Point.h"


Point calculateNewPoint(Point oldPoint, double alpha, double theta, double radius, int coefficient){
  Point p2;
  Point rotationCenter;
  rotationCenter.x = oldPoint.x-(coefficient*(radius*cos(alpha)));
  rotationCenter.y = oldPoint.y-(coefficient*(radius*sin(alpha)));
  double beta = alpha + coefficient*theta;
  p2.x = rotationCenter.x + coefficient*(cos(beta)*radius);
  p2.y = rotationCenter.y + coefficient*(sin(beta)*radius);
  return p2;
}
void calculateNewPos(Point leftWheel, Point rightWheel, double leftDist, double rightDist,Point* leftWheelResult, Point* rightWheelResult,  double EncoderDist){
	printf("\n");
	printf("Left Distance: %f\n", leftDist);
	printf("Right Distance: %f\n", rightDist);
	printf("Left Wheel X: %f\n", leftWheel.x);
	printf("Left Wheel Y: %f\n", leftWheel.y);
	printf("Right Wheel X: %f\n", rightWheel.x);
	printf("Right Wheel Y: %f\n", rightWheel.y);
  if(leftDist==rightDist){
    leftWheelResult->x = leftWheel.x;
    leftWheelResult->y = leftWheel.y + leftDist;
    rightWheelResult->x = rightWheel.x;
    rightWheelResult->y = rightWheel.y + leftDist;
	  printf("are the same \n");
	  printf("New Left Wheel X: %f\n", leftWheelResult->x);
	  printf("New Left Wheel Y: %f\n", leftWheelResult->y);
  	printf("New Right Wheel X: %f\n", rightWheelResult->x);
	  printf("New Right Wheel Y: %f\n", rightWheelResult->y);
    return;
  }
  int coefficient = 1;
  double slope = (rightWheel.y-leftWheel.y)/(rightWheel.x-leftWheel.x);
  //printf("right wheel y - left wheel y = %f\n", (rightWheel.y-leftWheel.y));
  //printf("right wheel x- left wheel x = %f\n", (rightWheel.x-leftWheel.x));

  double leftRad,rightRad,theta;
  if(rightDist>leftDist){
    leftRad = (leftDist*EncoderDist)/(rightDist-leftDist);
    rightRad = leftRad+EncoderDist;
    theta = leftDist/leftRad;
  }else{
    coefficient *= -1;
    rightRad = (rightDist*EncoderDist)/(leftDist-rightDist);
    leftRad = rightRad+EncoderDist;
    theta = leftDist/leftRad;
  }
  *leftWheelResult = calculateNewPoint(leftWheel,atan(slope),theta,leftRad,coefficient);
  *rightWheelResult = calculateNewPoint(rightWheel,atan(slope),theta,rightRad,coefficient);
  double dX = (leftWheel.x+rightWheel.x)/2 - (leftWheelResult->x+rightWheelResult->x)/2;
  double dY = (leftWheel.y+rightWheel.y)/2 - (leftWheelResult->y+rightWheelResult->y)/2;

  printf("Mid change %f\n", sqrt((dX*dX)+(dY*dY)));
  printf("New Left Wheel X: %f\n", leftWheelResult->x);
  printf("New Left Wheel Y: %f\n", leftWheelResult->y);
  printf("New Right Wheel X: %f\n", rightWheelResult->x);
  printf("New Right Wheel Y: %f\n", rightWheelResult->y);
  
}