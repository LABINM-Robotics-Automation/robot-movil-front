# React + Vite

This template provides a minimal setup to get React working in Vite with HMR and some ESLint rules.

Currently, two official plugins are available:

### Steps to run the project

```bash
# run the front
npm run dev

# run the websocket server in ROS
roslaunch rosbridge_server rosbridge_websocket.launch

# run some image publisher like zed2i or a local (see example)
cd ~/mobile_robot_iot/publisher/publisher_image/src
rosrun mobile_robot_iot image_publisher_src.py
```


- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react/README.md) uses [Babel](https://babeljs.io/) for Fast Refresh
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh
