// circle.cpp (Implementation file)
#include "circle.h"
#include <cmath>

const double PI = 3.14159; // Define the value of PI as a constant

Circle::Circle(double rad) : radius(rad) {} // Constructor

void Circle::setRadius(double rad)
{
    radius = rad;
}

double Circle::calculateArea() const
{
    return PI * pow(radius, 2); // Formula to calculate the area of a circle
}

double Circle::calculatePerimeter() const
{
    return 2 * PI * radius; // Formula to calculate the circumference (perimeter) of a circle
}

void Circle::setArea(double area)
{
    radius = sqrt(area / PI);// You can implement this method based on your requirements
}

void Circle::setPerimeter(double perimeter)
{
    radius = perimeter / (2 * PI);// You can implement this method based on your requirements
}
