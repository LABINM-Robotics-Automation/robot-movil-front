import './App.css'
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const ImageSubscriber = () => {
  const [imageSrc, setImageSrc] = useState('');

  useEffect(() => {
    // Connect to the ROS WebSocket server
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090' // Update with your ROS WebSocket server URL
    });

    ros.on('connection', () => {
      console.log('Connected to ROS');
    });

    ros.on('error', (error) => {
      console.log('Error connecting to ROS: ', error);
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed');
    });

    // Create a ROS topic listener for the compressed image
    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      // name: '/zed2i/zed_node/right_raw/image_raw_color/compressed', // Update with your topic name
      name: '/image_topic/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

    imageTopic.subscribe((message) => {
      const binaryString = atob(message.data);  // Decode base64 data
      const len = binaryString.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }

      // Create a Blob from the byte array
      const blob = new Blob([bytes], { type: 'image/jpeg' });

      // Create an object URL from the blob
      const imageUrl = URL.createObjectURL(blob);

      // Create an offscreen canvas for resizing
      const img = new Image();
      img.src = imageUrl;

      img.onload = () => {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');

        // Set the canvas size to the target size (e.g., half of the original size)
        const MAX_WIDTH = 800; // Set your desired max width
        const MAX_HEIGHT = 600; // Set your desired max height
        let width = img.width;
        let height = img.height;

        // Calculate new dimensions maintaining the aspect ratio
        if (width > height) {
          if (width > MAX_WIDTH) {
            height *= MAX_WIDTH / width;
            width = MAX_WIDTH;
          }
        } else {
          if (height > MAX_HEIGHT) {
            width *= MAX_HEIGHT / height;
            height = MAX_HEIGHT;
          }
        }

        // Resize the canvas to the new dimensions
        canvas.width = width;
        canvas.height = height;

        // Draw the image onto the canvas
        ctx.drawImage(img, 0, 0, width, height);

        // Convert the canvas back to a Base64-encoded data URL
        const resizedDataUrl = canvas.toDataURL('image/jpeg');

        // Set the resized image as the source
        setImageSrc(resizedDataUrl); // Assuming `setImageSrc` is your state setter for the image source

        // Release the object URL after use
        URL.revokeObjectURL(imageUrl);
      };
    });

    // Cleanup on component unmount
    return () => {
      imageTopic.unsubscribe();
      ros.close();
    };
  }, []);

  return (
    <div>
      {imageSrc ? (
        <img src={imageSrc} alt="Received from ROS" style={{ width: '100%', height: 'auto', paddingTop: '10px' }} />
      ) : (
        <p>Loading image...</p>
      )}
    </div>
  );


};

export default ImageSubscriber;

