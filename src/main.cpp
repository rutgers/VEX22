#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef AUTON_CPP
#define AUTON_CPP
#include "auton_util.cpp"
#endif

/*

    selector::auton == 1 : Red Front
    selector::auton == 2 : Red Back
    selector::auton == 3 : Do Nothing
    selector::auton == -1 : Blue Front
    selector::auton == -2 : Blue Back
    selector::auton == -3 : Do Nothing
    selector::auton == 0 : Skills
*/


#include "autoSelect/selection.h"

#define SKAR_1 2
#define SKAR_2 3

<<<<<<< HEAD
#define SKILLS true
#define BUILD_TARGET SKAR_3 //SKAR_1 or SKAR_2
//#define AUTON_SIDE RED //RED OR BLUE
// #if BUILD_TARGET == SKAR_1
//     #include "SKAR_1.cpp"
// #endif
=======
// Build Target
#define BUILD_TARGET SKAR_1 //SKAR_1 or SKAR_2

// Initial speed for auton
#define AUTON_INIT 165

// Build Target
#if BUILD_TARGET == SKAR_1
    // Tank Control Swap
    #define TANK true // TRUE is Tank Drive and FALSE is Regular
    //bool tank = (bool) getenv("TANK");
    #include "SKAR_1.cpp"
#endif
>>>>>>> 3b94b8436ad9986bf11aef84920f340e042fac94
#if BUILD_TARGET == SKAR_2
    #include "SKAR_2.cpp"
#endif
#if BUILD_TARGET == SKAR_3
    #include "SKAR_3.cpp"
#endif
