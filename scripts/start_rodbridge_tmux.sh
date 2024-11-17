#!/bin/bash

# Nombre de la sesión tmux
SESSION_NAME="rosbridge_server"

# Verifica si la sesión tmux ya existe
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "La sesión tmux '$SESSION_NAME' ya está en ejecución."
    exit 0
fi

# Crear una nueva sesión de tmux y ejecutar el comando
echo "Iniciando rosbridge_server en una nueva sesión de tmux..."
tmux new-session -d -s $SESSION_NAME "roslaunch rosbridge_server rosbridge_websocket.launch"

echo "Sesión tmux '$SESSION_NAME' iniciada."

