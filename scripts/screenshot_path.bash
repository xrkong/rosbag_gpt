#!/bin/bash

# Specify the area for the screenshot
X=50  # X coordinate
Y=450  # Y coordinate
WIDTH=1750  # Width
HEIGHT=650  # Height
OUTPUT="screenshot.png"  # Output file name

# open webpage
HTML_FOLDER="./output"
for file in "$HTML_FOLDER"/*.html
do
    filename=$(basename "$file")
    filename="${filename%.*}"  # Remove the file extension
    output="${filename}.png"  # Output file name

    xdg-open "$file"
    echo "xdg-open" "$file"
    sleep 3
    # sudo apt-get install imagemagick
    # Take a screenshot of the specified area
    import -window "$(xdotool getwindowfocus)" -crop "${WIDTH}x${HEIGHT}+${X}+${Y}" "$HTML_FOLDER"/"$output"
    #scrot -a {X},${Y},${WIDTH},${HEIGHT} "$HTML_FOLDER"/"$output"
    sleep 1
done
