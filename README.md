# robot-movil-front

Frontend page for the robot movil, build over **React** framework.

### Steps to run the project

```bash
# clone and enter to project
git clone https://github.com/LABINM-Robotics-Automation/robot-movil-front.git
cd robot-movil-front

# install dependencies and start front
npm i
npm run dev

# run the websocket server in ROS
roslaunch rosbridge_server rosbridge_websocket.launch

# run some image publisher like zed2i or a local (see example)
cd ~/mobile_robot_iot/publisher/publisher_image/src
rosrun mobile_robot_iot image_publisher_src.py
```

### Procedimientos para configurar el servicio

Estos son los pasos para configurar el inicio automático del servidor `robot-movil-front` en un entorno Linux:

1. Dale permisos de ejecución al script `start_front_server_in_laptop.sh`:

    El script de inicio se encuentra en la carpeta `scripts/start_front_server_in_laptop.sh`

    ```bash
    cd react-front-server
    chmod +x /scripts/start_front_server_in_laptop.sh
    ```
2. Configura el inicio automático usando systemd:

    Crea un archivo de servicio de systemd:

    ```bash
    sudo nano /etc/systemd/system/robot_movil_front_server.service
    ```

    Añade el siguiente contenido al archivo:

    ```
    [Unit]
    Description=React Server in tmux
    After=network.target

    [Service]
    Type=forking
    ExecStart=/home/tu_usuario/robot-movil-front/scripts/start_front_server.sh 

    User=tu_usuario
    Environment=DISPLAY=:0

    [Install]
    WantedBy=multi-user.target
    ```
    Asegúrate de reemplazar "tu_usuario" con tu nombre de usuario real.

3. Recarga el daemon de systemd para que reconozca el nuevo servicio:

    ```bash
    sudo systemctl daemon-reload
    ```

4. Habilita el servicio para que se inicie automáticamente en el arranque:

    ```bash
    sudo systemctl enable robot_movil_front_server.service
    ```

5. Inicia el servicio manualmente por primera vez (o reinicia el sistema):

    ```bash
    sudo systemctl start robot_movil_front_server.service
    ```

6. Verifica que el servicio esté funcionando correctamente:

    ```bash
    sudo systemctl status robot_movil_front_server.service
    ```

    Con estos pasos, tu servidor React debería iniciarse automáticamente en una sesión de tmux cada vez que tu sistema Linux se inicie. Puedes acceder a la sesión de tmux en cualquier momento usando:

    ```bash
    tmux attach-session -t robot_movil_front_server
    ```

    Recuerda que necesitarás tener tmux y Node.js (con npm) instalados en tu sistema para que esto funcione correctamente.

