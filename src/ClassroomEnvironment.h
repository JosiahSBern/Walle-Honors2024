#ifndef CLASSROOMENVIRONMENT_H
#define CLASSROOMENVIRONMENT_H

#include "Environment.h" // Include base Environment class

class ClassroomEnvironment : public Environment {
public:
    // Constructor
    ClassroomEnvironment(rclcpp::Node::SharedPtr node);
    void drawClassroom(); 
    
private:
    // Add additional methods or properties if necessary
};

#endif 
