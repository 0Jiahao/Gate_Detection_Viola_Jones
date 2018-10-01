#!/bin/bash

mkdir Vision/build
cd Vision/build
cmake ..
cmake --build .

cd ../../

mkdir outerloop/build
cd outerloop/build
cmake ..
cmake --build .
