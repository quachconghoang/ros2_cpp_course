#include <iostream>

// Pointers are essential to C++ and are used to store the memory address of another variable.
// They are declared by using the asterisk (*) before the variable name.
// The data type of the pointer must be the same as the data type of the variable you're storing the address of.
// When passed as a parameter to a function, the function can modify the value of the variable by using the pointer.

int main () {

    // Ex01
//    * Assign to variable x (of type double) the value 41012
    double x = 41012;
    std::cout << "x = " << x << std::endl;

//    * Use a pointer ip to point to x (why type is ip?)
        double *ip = &x;

//    * Print the value of what ip is pointing to
        std::cout << *ip << std::endl;

//    * Print the value of ip
        std::cout << ip << std::endl;

//    * Make variable y a reference to x (what type is y?)
        double &y = x;

//    * Print the value of what y is referencing to
        std::cout << y << std::endl;

//    * Create a double z of value 1
        double z = 1;

//    * Use pointer ip to point to z
        ip = &z;

//    * Assign z the value 100
        z = 100;

//    * Print the value of what ip is pointing to
        std::cout << *ip << std::endl;

//    * Print the value of x
        std::cout << x << std::endl;

//    * Print the value of y
        std::cout << y << std::endl;

//    * Print the value of z
        std::cout << z << std::endl;

    return 0;
}
