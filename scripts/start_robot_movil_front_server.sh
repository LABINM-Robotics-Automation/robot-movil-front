#!/bin/bash

# Set your project directory
PROJECT_DIR="/home/labinm-jetson/xavier_ssd/robot-movil-front"

# Set the name for the tmux session
SESSION_NAME="robot_movil_front_server"

# Change to the project directory
cd "$PROJECT_DIR" || exit 

# Check if the tmux session already exists
if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    tmux new-session -d -s "$SESSION_NAME" "npm run dev"
    echo "Django server started in tmux session: $SESSION_NAME"
else
    echo "Tmux session $SESSION_NAME already exists."
fi
