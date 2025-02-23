import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import websocketUrl from './axiosInstance/websocketInstance'

function RecordingIndicator({
  style = { textAlign: 'center', marginTop: '10px' } 
}) {
  const [recordingActive, setRecordingActive] = useState(false);

  useEffect(() => {

    const ros = new ROSLIB.Ros(websocketUrl);  

    const checkRecordingNode = () => {
      ros.getNodes((nodes) => {
        // Check if the /rosbag_record node is in the list of active nodes
        const isActive = nodes.includes('/rosbag_record');
        setRecordingActive(isActive);
      }, (error) => {
        console.error('Error checking nodes:', error);
      });
    };

    // Initial check and set up interval
    checkRecordingNode();
    const intervalId = setInterval(checkRecordingNode, 1000); // Check every second

    // Clean up the interval on component unmount
    return () => clearInterval(intervalId);
  }, []);

  return (
    <div style={style}>
      <p style={{ color: recordingActive ? 'green' : 'red' }}>
        Recording Status: {recordingActive ? 'Active' : 'Inactive'}
      </p>
    </div>
  );
}

export default RecordingIndicator;

