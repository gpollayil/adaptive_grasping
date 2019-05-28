#ifndef REVERSEPRIORITYMANAGER_H
#define REVERSEPRIORITYMANAGER_H

/*
    REVERSE PRIORITY MANAGER CLASS
    This object is a manager for a set of tasks and performs reverse priority algorithm.
*/

// Basic Includes

// Custom Includes
#include "basicTask.h"


namespace adaptive_grasping {

class reversePriorityManager {

public:

    // Default Constructor
    reversePriorityManager();

    // Overloaded Constructor
    // reversePriorityManager();

    // Destructor
    ~reversePriorityManager();

private:

    // Set of tasks ordered by priority
    std::vector<basicTask> task_set_;

};

}

#endif // REVERSEPRIORITYMANAGER_H
