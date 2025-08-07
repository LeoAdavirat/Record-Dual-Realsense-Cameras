#!/bin/bash
echo "Starting indefinite recording loop..."
echo "A new recording session will begin each time the previous one finishes."
echo "Press Ctrl+C to exit the script."

while true
do
    python3 multicam_recorder.py --length 60 

    echo "Session complete. Restarting..."
    sleep 0
done
