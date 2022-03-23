#define SKAR_1 2
#define SKAR_2 3
#define SKAR_3 4

#define SKILLS true
#define BUILD_TARGET SKAR_3 //SKAR_1 or SKAR_2
//#define AUTON_SIDE RED //RED OR BLUE
// #if BUILD_TARGET == SKAR_1
//     #include "SKAR_1.cpp"
// #endif
#if BUILD_TARGET == SKAR_2
    #include "SKAR_2.cpp"
#endif
#if BUILD_TARGET == SKAR_3
    #include "SKAR_3.cpp"
#endif