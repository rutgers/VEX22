#pragma once

#include <string>

//selector configuration
#define HUE 360
#define DEFAULT 0 // 2 is for the right side and 1 is for the left side
#define AUTONS "Left", "Right RightYellow", "Right CenterYellow"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
