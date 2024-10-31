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

