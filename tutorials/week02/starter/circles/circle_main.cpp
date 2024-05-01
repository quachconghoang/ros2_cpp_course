// circle_main.cpp
#include <iostream>
#include <vector>
#include "circle.h"
#include <random>

double combined_Area(const std::vector<Circle>& circles) {
    double total_area = 0.0;
    for (const auto& Circle : circles) 
    {
        total_area += Circle.calculateArea();}

        return total_area;
}


int main()
{
    std::vector<Circle> circles; // Create a vector to store circles

    // Create three circles with different radii
    circles.push_back(Circle(1.0));
    circles.push_back(Circle(2.0));
    circles.push_back(Circle(5.0));

    // Calculate and display the area and perimeter for each circle
    for (const auto& circle : circles)
    {
        double area = circle.calculateArea();
        double perimeter = circle.calculatePerimeter();
        std::cout << "Area: " << area << "\n";
        std::cout << "Perimeter: " << perimeter << "\n\n";

    }

    //random genrator
    std::vector<Circle>Circles2;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution (1.0, 10.0);

    int number_of_circles;
    std::cout << "enter number of circles: ";
    std::cin >> number_of_circles;

    for (int i = 0; i < number_of_circles; ++i) {
        Circles2.emplace_back(distribution(generator));
    }

    std::cout << "combined area: " << combined_Area(Circles2) << std::endl;

    return 0;
}