#include "structDefs.hpp"

Point::Point(double inX, double inY){
    x = inX;
    y= inY;
};
double Point::operator-(const Point& rhs) {
	return this->x - rhs.x + this->y - rhs.y;
}