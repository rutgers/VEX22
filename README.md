# Rutgers Vex U Programming Source Code 

## Setup your Environment (as of 2022)
1. Install the Integrated Development Environment
    + To start programming make sure you have Vex pros installed on your computer. To install pros first go to the <a href="https://pros.cs.purdue.edu/v5/getting-started/installation.html">website</a> to install the program. When you are on the website follow the instructions to install the ide on your respective OS.
        + **Note: When you have installed it, uploading on MacOS has seen some issues. So if your a mac user you have access to one of the lab computers.**
2. Setup Vex Pros with your IDE
    + When Vex PROS is installed, download <a href="https://code.visualstudio.com/download">VSCode</a> for ease of use. Once VSCode is installed, make sure to setup the extension.
        + On Windows press Ctrl+Shift+X and type in PROS (should be the first one)
        + Install that extension and once that is done we can finally start programming
3. Once PROS is installed and VSCode is setup we can start programming 

## How to start coding
Now that we have our Integrated Development Environment ready, we can start programming 😀!!!! To start programming, first we should start a new project. To start a new project go to VSCode and go to install a new plugin. In the extensions section, type in `Vex Pros` and when it loads it will be the first extension you need to install. Once that is installed you should see the Vex Pros logo <img src="https://user-images.githubusercontent.com/22580992/123097191-e198b480-d3fd-11eb-903c-4c267f59fac1.png" width=40px height=30px/>. In that extension, click `create project` where it will allow you to create the project in the directory of your choosing. Once that is done, you can open the project and start coding.

## Setting up your main file
Once you are the part where you can start coding, let's set up a build function to start coding to multiple bots all in one ide <img src="https://i.ytimg.com/vi/P9PD8V_iyxQ/maxresdefault.jpg" width=40px height=30px/>. Go to your main.cpp file where we will setup the building capability where all you need to do to build to a certain bot (i.e: SKAR_1, SKAR_2, ...). Now that your in your main file 

```cpp

/*
 
 MACRO is the macro name that is all in caps usually 
 
 library hpp file is the library that you want to define
 
 <MACRO NAME FOR SKAR BOT> is usually in all caps for example SKAR_1 (for the first skar bot) 
    - It can also be BIG_BOT, SMALL_BOT (all up to your discretion)
 
 */

#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef MACRO
#define MACRO
#include library hpp file
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

#define <MACRO NAME FOR SKAR BOT> <Integer>

// Define Target
#define BUILD_TARGET <MACRO NAME FOR SKAR BOT> //SKAR_1 or SKAR_2

// Build Target
#if BUILD_TARGET == <MACRO NAME FOR SKAR BOT 1>
    #include "SKAR_1.cpp"
#endif
#if BUILD_TARGET == <MACRO NAME FOR SKAR BOT 1>
    #include "SKAR_2.cpp"
#endif
#if BUILD_TARGET == <MACRO NAME FOR SKAR BOT 1>
    #include "SKAR_3.cpp"
#endif
```
