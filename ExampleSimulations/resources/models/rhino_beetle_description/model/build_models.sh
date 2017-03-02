#!/bin/bash

BUILD_DIRECTORY=./build
mkdir -p ${BUILD_DIRECTORY}/sdf
#mkdir -p ${BUILD_DIRECTORY}/urdf

# generate sdf models
gzsdf print urdf/model.urdf > sdf/model.sdf

#echo "These files are autogenerated" > urdf/README.md
echo "These files are autogenerated" > sdf/README.md
