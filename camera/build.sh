#! /bin/bash

Qt_PATH=$HOME/Qt/5.13.0/gcc_64

mkdir build && cd build

cmake -DCMAKE_PREFIX_PATH=$Qt_PATH ..
make -j4

cd ..
