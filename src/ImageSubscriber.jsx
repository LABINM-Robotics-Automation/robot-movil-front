import './App.css'
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import websocketUrl from './axiosInstance/websocketInstance'

function PlaceholderImage({
    imageHeight = 100,
    imageWidth = 100
}) {
  const [placeholderSrc, setPlaceholderSrc] = useState('');
  useEffect(() => {
    // Create a canvas element
    const canvas = document.createElement('canvas');

    canvas.width = imageWidth;
    canvas.height = imageHeight;
    const ctx = canvas.getContext('2d');

    // Set background color
    ctx.fillStyle = '#333';  // Dark gray color
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Set text properties
    ctx.fillStyle = '#FFF';  // White color for text
    ctx.font = 'bold 24px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    // Draw text
    ctx.fillText('Not Available', canvas.width / 2, canvas.height / 2);

    // Convert canvas to an image source
    setPlaceholderSrc(canvas.toDataURL('image/png'));
  }, []);

  return (
    <div>
      <img src={placeholderSrc} alt="Not Available" style={{ width: '100%', height: 'auto', paddingTop: '10px', opacity: 0.5 }} />
    </div>
  );
}

const ImageSubscriber = ({
    cameraActive,
    windowHeight = 100,
    windowWidth = 100
}) => {
  const [imageSrc, setImageSrc] = useState('');

  useEffect(() => {

    if (!cameraActive){
      setImageSrc('')
      return
    }
    
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
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

    const imageTopic = new ROSLIB.Topic({
      ros: ros,
      // name: '/zed2i/zed_node/right_raw/image_raw_color/compressed',
      name: '/processed_image/compressed',
      messageType: 'sensor_msgs/CompressedImage'
    });

    imageTopic.subscribe((message) => {
      const binaryString = atob(message.data);  // Decode base64 data
      const len = binaryString.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
      const blob = new Blob([bytes], { type: 'image/jpeg' });
      const imageUrl = URL.createObjectURL(blob);
      const img = new Image();
      img.src = imageUrl;

      img.onload = () => {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        const MAX_WIDTH = windowWidth; // Set your desired max width
        const MAX_HEIGHT = windowHeight; // 9000; // Set your desired max height
        let width = img.width;
        let height = img.height;

        if (width > height) {
          if (width > MAX_WIDTH){
            height *= MAX_WIDTH / width;
            width = MAX_WIDTH;
          }
        } else {
          if (height > MAX_HEIGHT) {
            width *= MAX_HEIGHT / height;
            height = MAX_HEIGHT;
          }
        }
        
        canvas.width = width;
        canvas.height = height;
        ctx.drawImage(img, 0, 0, width, height);
        const resizedDataUrl = canvas.toDataURL('image/jpeg');
        setImageSrc(resizedDataUrl); // Assuming `setImageSrc` is your state setter for the image source
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
      {imageSrc ? 
        (<
          img src={imageSrc} alt="Received from ROS" 
          style={{ width: '100%', height: 'auto', paddingTop: '10px' }} 
        />)
        : 
        (<
          PlaceholderImage  imageHeight = {windowHeight} imageWidth = {windowWidth}  
        />)
      }
    </div>
  );


};

export default ImageSubscriber;
