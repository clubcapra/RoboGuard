#!/usr/bin/env bash

BASEDIR=$(dirname "$0")

BUILD_PATH=${BASEDIR}/../.pio/libdeps/with_micro_ros/micro_ros_platformio
EXPORT_PATH=${BASEDIR}/../lib/micro_ros_platformio_precompiled

LIBMICROROS_FILE=${BUILD_PATH}/libmicroros/libmicroros.a

RED='\033[0;31m'
NC='\033[0m'

if ! test -f "$LIBMICROROS_FILE"; then
    echo "ERROR : Can't find the built library libmicroros.a make sure to build with_micro_ros first"
    exit 
fi

if ! test -d "${EXPORT_PATH}"; then
    echo "Creating ${EXPORT_PATH}"
    mkdir ${EXPORT_PATH}
else
    echo "Deleting contents of ${EXPORT_PATH}..."
    rm -rf ${EXPORT_PATH}/*
fi

mkdir ${EXPORT_PATH}/src
mkdir ${EXPORT_PATH}/src/cortex-m4

echo "Copying files to ${EXPORT_PATH}..."

cp ${LIBMICROROS_FILE} ${EXPORT_PATH}/src/cortex-m4/
cp -a ${BUILD_PATH}/libmicroros/include/* ${EXPORT_PATH}/src
cp -a ${BUILD_PATH}/platform_code/arduino/serial/* ${EXPORT_PATH}/src
cp  ${BUILD_PATH}/platform_code/arduino/clock_gettime.cpp ${EXPORT_PATH}/src
cp  ${BUILD_PATH}/platform_code/micro_ros_platformio.h ${EXPORT_PATH}/src

echo "Done!"
