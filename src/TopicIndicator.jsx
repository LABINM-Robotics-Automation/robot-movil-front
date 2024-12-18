import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import websocketUrl from './axiosInstance/websocketInstance.jsx'

function CameraStatusIndicator({
  style = { textAlign: 'center', marginTop: '10px' }
}) {
  const [cameraStatus, setCameraStatus] = useState(null); // true = active, false = inactive
  const [loading, setLoading] = useState(true);

  const ros = new ROSLIB.Ros(websocketUrl);  

  const checkCameraStatus = () => {
    const topicName = '/zed2i/zed_node/right_raw/image_raw_color';
    const nodeName = '/zed2i/zed_node';

    // Check if the topic is active
    ros.getTopics((result) => {
      const topics = result.topics || [];
      const topic_active = topics.includes(topicName);

      // Check if the node is active
      ros.getNodes((nodes) => {
        const node_active = nodes.includes(nodeName);

        // Update camera status: true only if both node and topic are active
        setCameraStatus(node_active && topic_active);
        setLoading(false);
      });
    }, (error) => {
      console.error('Error checking topic:', error);
      setCameraStatus(false);
      setLoading(false);
    });
  };

  // Use useEffect to set up a periodic check every 1 second
  useEffect(() => {
    // Initial check
    checkCameraStatus();

    // Set up interval for periodic checks
    const intervalId = setInterval(checkCameraStatus, 1000); // Check every 1 second

    // Cleanup on component unmount
    return () => clearInterval(intervalId);
  }, []);



  return (
    <div style={style}>
      {loading ? (
        <p>Checking...</p>
      ) : (
        <p style={{ color: cameraStatus ? 'green' : 'red' }}>
          Camera Status: {cameraStatus ? 'Active' : 'Inactive'}
        </p>
        )}
    </div>
  );
}

export default CameraStatusIndicator;

