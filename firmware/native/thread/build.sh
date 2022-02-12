#!/bin/sh -e

## vars
export PROJECT_DIR=$(dirname $0)
export BUILD_DIR=$PROJECT_DIR/build

## configure
mkdir -p $BUILD_DIR
cmake -B $BUILD_DIR -S $PROJECT_DIR >/dev/null
## build
cmake --build $BUILD_DIR -- -j$(nproc) >/dev/null
## run
$BUILD_DIR/app
