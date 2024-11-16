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
  
    console.log('webscoket: ', websocketUrl)
    const ros = new ROSLIB.Ros(websocketUrl);

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
      name: '/zed2i/zed_node/right_raw/image_raw_color', // Uncompressed topic
      messageType: 'sensor_msgs/Image'
    });

 
imageTopic.subscribe((message) => {
  const { width, height, encoding } = message;
  const binaryData = Uint8Array.from(atob(message.data), char => char.charCodeAt(0));

  // Create a temporary canvas
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  canvas.width = width;
  canvas.height = height;

  const imageData = ctx.createImageData(width, height);

  if (encoding === 'bgra8') {
    for (let i = 0, j = 0; i < binaryData.length; i += 4, j += 4) {
      imageData.data[j] = binaryData[i + 2];     // Red
      imageData.data[j + 1] = binaryData[i + 1]; // Green
      imageData.data[j + 2] = binaryData[i];     // Blue
      imageData.data[j + 3] = binaryData[i + 3]; // Alpha
    }
  } else {
    console.error(`Unsupported encoding: ${encoding}`);
    return;
  }

  ctx.putImageData(imageData, 0, 0);

  // Convert canvas to a Blob
  canvas.toBlob((blob) => {
    if (blob) {
      createImageBitmap(blob).then((imageBitmap) => {
        // Now you can use the ImageBitmap to render or resize
        const outputCanvas = document.createElement('canvas');
        const outputCtx = outputCanvas.getContext('2d');

        const MAX_WIDTH = windowWidth; // Set your desired max width
        const MAX_HEIGHT = windowHeight; // Set your desired max height

        let scaleWidth = width;
        let scaleHeight = height;

        if (width > height && width > MAX_WIDTH) {
          scaleHeight = (MAX_WIDTH / width) * height;
          scaleWidth = MAX_WIDTH;
        } else if (height > width && height > MAX_HEIGHT) {
          scaleWidth = (MAX_HEIGHT / height) * width;
          scaleHeight = MAX_HEIGHT;
        }

        outputCanvas.width = scaleWidth;
        outputCanvas.height = scaleHeight;

        outputCtx.drawImage(imageBitmap, 0, 0, scaleWidth, scaleHeight);

        const resizedDataUrl = outputCanvas.toDataURL('image/jpeg');
        setImageSrc(resizedDataUrl); // Assuming `setImageSrc` is your state setter for the image source
      }).catch((error) => {
        console.error('Error creating ImageBitmap:', error);
      });
    } else {
      console.error('Failed to create blob from canvas.');
    }
  }, 'image/jpeg'); // Specify the desired format
});


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
