#pragma once
struct Point {
	//Designed to be used in inch, inch
	double x;
	double y;
    Point(double inX, double inY);
    Point() = default;
	double operator-(const Point& rhs);
};

struct resetStruct {
	bool switchPressed;
	Point pointReset;
	bool posReset;
	resetStruct(Point resetPoint);
};