#!/bin/bash

if [ $1 == "flake8-kiwidrive" ]; then
    flake8 kiwidrive
elif [ $1 == "flake8-tiredrive" ]; then
    flake8 tiredrive
elif [ $1 == "tiredrive-unittests" ]; then
    python tiredrive/robot.py test
elif [ $1 == "kiwidrive-unittests" ]; then
    python kiwidrive/robot.py test
fi
