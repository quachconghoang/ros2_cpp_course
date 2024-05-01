#include "shapeprocessing.h"
#include <iostream>

ShapeProcessing::ShapeProcessing(vector<Shape*> shape)
{
}

bool ShapeProcessing::checkIntersections(double x, double y){

auto it = shape_.begin();
while (it != shape_.end()){
  if( ((*it)->checkIntercept(x,y))) {
    std::cout<< "Remove: " << (*(*it)).getDescription() << std::endl;
    shape_.erase(it);
  }
  else {
    ++it; 
    }
  }
  return shape_.size()==0;
}

unsigned int ShapeProcessing::getNumShapes(){
  return shape_.size();
}