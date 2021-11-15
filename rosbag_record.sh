#!/bin/bash

MAIN_DIR=$1
PROJECT_NAME=$2
FOLDER_NAME=$(date +%Y%m%d_%H%M%S)

echo "Starting ros2 bag record into file '$MAIN_DIR/$PROJECT_NAME/$FOLDER_NAME'"

# ros2 bag record -a -o "$MAIN_DIR/$PROJECT_NAME/$FOLDER_NAME"
