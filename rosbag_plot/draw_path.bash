#!/bin/bash

# modify the following variables
DIRECTORY="/home/kong/dataset/eglinton-nov"
PYTHON_SCRIPT="./plot_ros2bag.py"

for file in "$DIRECTORY"/reg/*/*.mcap
do
    file_name=$(basename "$file")
    parent_dir=$(basename "$(dirname "$file")")
    rosbag_dir="$parent_dir/$file_name"
    incdent_file="$DIRECTORY"/inc/"$rosbag_dir"
    # echo 0 $file 1 $file_name 2 $parent_dir 3 $rosbag_dir 4 $incdent_file

    echo "$PYTHON_SCRIPT" "$file"
    python3 "$PYTHON_SCRIPT" "$file" -s "$incdent_file"
done

# remove html files smaller than 500k
find ./output -type f -size -500k -delete