#!/bin/bash

if [ $1 == "flake8-kiwidrive" ]; then
    flake8 kiwidrive
elif [ $1 == "flake8-tiredrive" ]; then
    flake8 tiredrive
elif [ $1 == "tiredrive-unittests" ]; then
    python tiredrive/robot.py test
elif [ $1 == "kiwidrive-unittests" ]; then
    if ! diff kiwidrive/parallel_generators.py tiredrive/parallel_generators.py > /dev/null; then
        echo "parallel_generators.py differ between kiwidrive and tiredrive!"
        exit 1
    fi
    python kiwidrive/robot.py test
fi
