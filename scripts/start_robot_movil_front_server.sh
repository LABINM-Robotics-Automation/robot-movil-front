#!/bin/bash

PROJECT_DIR="/home/labinm-jetson/xavier_ssd/robot-movil-front"
SESSION_NAME="robot_movil_front_server"
LOG_FILE="/tmp/robot_movil_front_server.log"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "Script started at $(date)"

cd "$PROJECT_DIR" || { echo "Error: Failed to change to project directory"; exit 1; }

start_tmux_session(){
        echo "Starting tmux session..."
        tmux new-session -d -s "$SESSION_NAME"
        tmux send-keys -t "$SESSION_NAME" "npm run dev" C-m
        echo "React server started in tmux session: $SESSION_NAME"
}

if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        start_tmux_session
else
        echo "Tmux session $SESSION_NAME already exists."
fi

echo "Waiting for npm to start..."
sleep 10

echo "Checking npm process..."

while tmux has-session -t "$SESSION_NAME" 2>/dev/null; do

        if ! tmux capture-pane -t "$SESSION_NAME" -p -S - | grep -q "npm run dev"; then
                echo "npm process not found in tmux session. Capturing pane content:"
                tmux capture-pane -t "$SESSION_NAME" -p
                echo "Restarting..."
                tmux kill-session -t "$SESSION_NAME"
                start_tmux_session
        else
                echo "npm process is running at $(date)"
        fi

        sleep 60
done

echo "Tmux session $SESSION_NAME has ended."
