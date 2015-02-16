#!/bin/bash

if [ $1 == "flake8-kiwidrive" ]; then
    flake8 kiwidrive
elif [ $1 == "flake8-tiredrive" ]; then
    flake8 tiredrive
elif [ $1 == "tiredrive-unittests"]; then
    python tiredrive/robot.py test tests.py
fi
