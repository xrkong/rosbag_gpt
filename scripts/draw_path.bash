#!/bin/bash

# modify the following variables
DIRECTORY="/home/kong/eglinton/2024_07"
PYTHON_SCRIPT="./plot_ros2bag.py"

# Check if at least two arguments were provided
# if [ $# -lt 2 ]; then
#   echo "Usage: $0 <arg1> <arg2>"
#   exit 1
# fi

# arg1="$1"
# arg2="$2"


for file in "$DIRECTORY"/inc/*/*.mcap
do
    root_dir="${file%inc/*/*.mcap}" # root_diction
    file_name=$(basename "$file") # rosbag2_incident_data.mcap
    parent_dir=$(basename "$(dirname "$file")") # rosbag2_incident_data
    rosbag_dir="$parent_dir"/"$file_name"
    incdent_file="$root_dir"snapshot/"$rosbag_dir"
    echo root_dir $root_dir
    echo file $file 
    echo file_name $file_name 
    echo parent_dir $parent_dir 
    echo rosbag_dir $rosbag_dir 
    echo incdent_file $incdent_file

    echo "$PYTHON_SCRIPT" "$file"
    python3 "$PYTHON_SCRIPT" "$file" -s "$incdent_file"
    #python3 "$PYTHON_SCRIPT" "$file"
done

echo "remove html files smaller than 500k"
find ./output -type f -name "*.html" -size -500k -delete
