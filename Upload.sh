#!/bin/bash
export AUTON_COLOR BLUE
export TANK true
prosv5 mu --slot 1 --name "Tank Blue"

export AUTON_COLOR RED
export TANK true
prosv5 mu --slot 2 --name "Tank Red"

export AUTON_COLOR RED
export TANK false
prosv5 mu --slot 3 --name "Tank Red"