#include <math.h>
#include< cstdio>
//k#include ".. \include\Point.h"


Point calculateNewPoint(Point oldPoint, double alpha, double theta, double radius, int coefficient){
  Point p2;
  Point rotationCenter;
  rotationCenter.x = oldPoint.x-(coefficient*(radius*cos(fabs(alpha))));
  rotationCenter.y = oldPoint.y+(radius*sin(fabs(alpha)));
  double beta = alpha + coefficient*theta;
  p2.x = rotationCenter.x + coefficient*(cos(beta)*radius);
  p2.y = rotationCenter.y + coefficient*(sin(beta)*radius);
  return p2;
}
void calculateNewPos(Point leftWheel, Point rightWheel, double leftDist, double rightDist,Point* leftWheelResult, Point* rightWheelResult,  double EncoderDist){
  if(leftDist==rightDist){
    leftWheelResult->x = leftWheel.x;
    leftWheelResult->y = leftWheel.y + leftDist;
    rightWheelResult->x = rightWheel.x;
    rightWheelResult->y = rightWheel.y + leftDist;
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

  
  
}