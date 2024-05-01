#include "gtest/gtest.h"
#include <iostream> //Not needed usually, here just for debugging
#include <vector>
#include <algorithm>

//header files needed from our libraries
#include "../rectangle.h"    // '../' folder bellow rectangle
#include "../circle.h"
#include "../triangle.h"
#include "../shapeprocessing.h"

TEST (IntercectTest, Circle) { //tets and subgroup circle 
    Circle circle(3.0);   //made a cirle with raduiis 3
    circle.setCentre(3.0,3.0);//Lets move centre to 3.0 3.0 so does not intercept 0,0 : but it could intercept 1,1
    ASSERT_FALSE(circle.checkIntercept(0,0));      //negtive test outcome
    ASSERT_TRUE(circle.checkIntercept(1.0,1.0));    //postive dest outcome 
}
 // make test for rectangle and triangle



int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
