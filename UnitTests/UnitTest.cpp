#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include "..\ChangeUpCode\include\Point.h"
#include "..\Shared\GeometryFunctions.cpp"

# define M_PI           3.14159265358979323846  /* pi */

using namespace std;
void newPointTester(double alpha, int coefficient, double endResultX, double endResultY, int numTest){
  Point calcNew;
  calcNew.x = 1;
  calcNew.y = 1;
  Point unitTest2Result = calculateNewPoint(calcNew,alpha,atan(1),sqrt(2),coefficient);
  if(fabs(unitTest2Result.x-endResultX)>.001||fabs(unitTest2Result.y-endResultY)>.001){
    printf("wrong unit test %d\n", numTest);
  }
}
int main()
{
    Point left;
    left.x = -6.484184;
     left.y = 3.627446;
    Point right;
    right.x = 6.515321;
  right.y = 3.514089;
  Point newLeft;
  newLeft.x = 0;
  newLeft.y = 0;
  Point newRight;
  newRight.x = 0;
  newRight.y = 0;
  newPointTester(atan(1),1,0,sqrt(2),1);
  newPointTester(atan(1),-1,2-sqrt(2),2,2);
  newPointTester(atan(-1),1,sqrt(2),2,3);
  newPointTester(atan(-1),-1,2,sqrt(2),4);
  //calculateNewPoint(calcNew,atan(-1),atan(-1),sqrt(2),1);
  //calculateNewPoint(calcNew,atan(-1),atan(-1),sqrt(2),-1);
  calculateNewPos(left, right, 3.92978, 3.778638, &newLeft, &newRight, 13);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0;  
   
  calculateNewPos(left, right, M_PI / 4, M_PI / 2, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 2, M_PI/4, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);  
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0;  
  calculateNewPos(left, right, M_PI / 2,M_PI / 4, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 4, M_PI/2, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0; 
  calculateNewPos(left, right, M_PI / 8, M_PI / 4, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 4, M_PI/8, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);  
  left.x = 0;
  left.y = 0;
  right.x = 1;
  right.y = 0; 
  calculateNewPos(left, right, M_PI / 4, M_PI / 8, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y);
  left = newLeft;
  right = newRight;
  calculateNewPos(left,right, M_PI / 8, M_PI/4, &newLeft, &newRight,1);
  printf("Left X: %f", newLeft.x);
  printf(" Left Y: %f\n", newLeft.y);
  printf("Right X: %f", newRight.x);
  printf(" Right Y: %f\n\n", newRight.y); 

}