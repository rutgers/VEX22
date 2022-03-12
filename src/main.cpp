#define SKAR_1 2
#define SKAR_2 3

// Build Target
#define BUILD_TARGET SKAR_1 //SKAR_1 or SKAR_2

// Auton Color
//#define AUTON_COLOR BLUE
char*auton_color = getenv("AUTON_COLOR");

// Skills color
#define SKILLS_COLOR YELLOW

// Initial speed for auton
#define AUTON_INIT 165

// Auton Side for Mobile Goals
#define AUTON_SIDE RED //RED OR BLUE

// Tank Control Swap
//#define TANK true // TRUE is Tank Drive and FALSE is Regular
bool tank = (bool) getenv("TANK");

// Skills Swap
#define SKILLS false // TRUE is Skills and FALSE is Regular

// Auton Number
#define AUTON_NUM 1 // 1 is small side and 2 is bigger side

// Build Target
#if BUILD_TARGET == SKAR_1
    #include "SKAR_1.cpp"
#endif
#if BUILD_TARGET == SKAR_2
    #include "SKAR_2.cpp"
#endif
#if BUILD_TARGET == SKAR_3
    #include "SKAR_3.cpp"
#endif
