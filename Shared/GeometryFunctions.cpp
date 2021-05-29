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


//Takes in the current points of the left wheel and right wheel as the first 2 parameters
//takes in the distance of the arc traveled for both encoders for the next 2 paremeters
//Takes in pointers so that it can return both wheels as a result
//Encoder dist is a constant that is the distance between the encoder wheels
void calculateNewPos(Point leftWheel, Point rightWheel, double leftDist, double rightDist,Point* leftWheelResult, Point* rightWheelResult,  double EncoderDist){
	printf("\n");
	printf("Left Distance: %f\n", leftDist);
	printf("Right Distance: %f\n", rightDist);
	printf("Left Wheel X: %f\n", leftWheel.x);
	printf("Left Wheel Y: %f\n", leftWheel.y);
	printf("Right Wheel X: %f\n", rightWheel.x);
	printf("Right Wheel Y: %f\n", rightWheel.y);

  /* This if statement handles the case that both of the distances are the same, because this means that there will be dividing by
  0 somewhere down the line, so this will handle the case because that means that it is going in a straight line.
  currently this code is incorrect because it just moves it straight upwares however long the wheels travel, but if the slope of the
  two wheels together wasn't 0, it will mess up and won't provide a correct answer
  */
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
  //The coeffecient is used to handle different directions and different quadrants, if the robot went to the left, the coefficient
  //turns negative to properly handle a negatie number and make sure that the end calculated points are added the correct way 
  //(left or right, up or down)
  int coefficient = 1;
  //The slope is used to calculate the alpha angle for the calculate new point position, the slope being calculate is the one between the
  //2 encoders on a graph. There is no need to handle the case of the x coordinates of both encoders being the same, as realistically 
  //the robot will never turn all the way sideways during a motion of going straight, therefore the case will never happen
  double slope = (rightWheel.y-leftWheel.y)/(rightWheel.x-leftWheel.x);
  //the left radius and right radius are interchangeable, one will be calculated and the other will just be extended by the constant
  double leftRad,rightRad,theta;

  //If the right distance is greater than the left distance that means that the center of rotation will be to the left of the the two
  //wheels, and if the right distance is larger than the left distance the calculations will just be reversed
  if(rightDist>leftDist){
    //the inside radius is calculated by 
    leftRad = (leftDist*EncoderDist)/(rightDist-leftDist);
    rightRad = leftRad+EncoderDist;
    //theta is in radians, it uses the definition of an arc of a circle which is just dividing the length of the arc by the radius
    //which gives you the angle
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