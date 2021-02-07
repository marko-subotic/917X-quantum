#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include "..\ChangeUpCode\include\Point.h"
#include "..\Shared\GeometryFunctions.cpp"

# define M_PI           3.14159265358979323846  /* pi */

using namespace std;
int main()
{
    Point left;
    left.x = 0;
     left.y = 0;
    Point right;
    right.x = 1;
  right.y = 0;
  Point newLeft;
  newLeft.x = 0;
  newLeft.y = 0;
  Point newRight;
  newRight.x = 0;
  newRight.y = 0;
  
  
  calculateNewPos(left, right, M_PI / 4, M_PI / 2, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 2, M_PI/4, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);  
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0;  
  calculateNewPos(left, right, M_PI / 2,M_PI / 4, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 4, M_PI/2, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0; 
  calculateNewPos(left, right, M_PI / 8, M_PI / 4, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 4, M_PI/8, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);  
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0; 
  calculateNewPos(left, right, M_PI / 4, M_PI / 8, &newLeft, &newRight,13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 8, M_PI/4, &newLeft, &newRight,14);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y); 
}