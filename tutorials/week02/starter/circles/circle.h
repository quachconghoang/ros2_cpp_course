// circle.h (Header file)
#ifndef CIRCLE_H
#define CIRCLE_H

class Circle
{
private:
    double radius; // Private member variable to store the radius

public:
    Circle(double rad); // Constructor to initialize the Circle object with a radius
    void setRadius(double rad); // Method to set the radius
    double calculateArea() const; // Method to calculate the area of the circle
    double calculatePerimeter() const; // Method to calculate the circumference (perimeter) of the circle
    void setArea(double area); // Method to set the area
    void setPerimeter(double perimeter); // Method to set the perimeter
};

#endif // CIRCLE_H
