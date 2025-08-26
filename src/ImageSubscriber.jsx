import React, { useRef, useEffect } from 'react';
import ROSLIB from 'roslib';
import websocketUrl from './axiosInstance/websocketInstance';

const drawPlaceholder = (ctx, width, height) => {
  ctx.fillStyle = '#333';
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = '#FFF';
  ctx.font = 'bold 24px Arial';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('Not Available', width / 2, height / 2);
};

const ImageSubscriber = ({
  cameraActive,
  windowHeight = 100,
  windowWidth = 100,
}) => {
  const canvasRef = useRef(null);

  // Solo dibuja el placeholder si la cámara se desactiva
  useEffect(() => {
    if (!cameraActive) {
      const canvas = canvasRef.current;
      if (canvas) drawPlaceholder(canvas.getContext('2d'), canvas.width, canvas.height);
    }
  }, [cameraActive, windowHeight, windowWidth]);

  useEffect(() => {
    if (!cameraActive) return;

    const ros = new ROSLIB.Ros(websocketUrl);

    const imageTopic = new ROSLIB.Topic({
      ros,
      name: '/processed_image/compressed',
      messageType: 'sensor_msgs/CompressedImage',
    });

    imageTopic.subscribe((message) => {
      const binaryString = atob(message.data);
      const len = binaryString.length;
      const bytes = new Uint8Array(len);
      for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }
      const blob = new Blob([bytes], { type: 'image/jpeg' });
      const url = URL.createObjectURL(blob);
      const img = new window.Image();
      img.onload = () => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        let drawWidth = img.width;
        let drawHeight = img.height;

        // Ajuste de tamaño proporcional
        const maxW = windowWidth;
        const maxH = windowHeight;
        if (drawWidth > drawHeight) {
          if (drawWidth > maxW) {
            drawHeight *= maxW / drawWidth;
            drawWidth = maxW;
          }
        } else {
          if (drawHeight > maxH) {
            drawWidth *= maxH / drawHeight;
            drawHeight = maxH;
          }
        }
        canvas.width = drawWidth;
        canvas.height = drawHeight;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(img, 0, 0, drawWidth, drawHeight);
        URL.revokeObjectURL(url);
      };
      img.onerror = () => {
        URL.revokeObjectURL(url);
      };
      img.src = url;
    });

    return () => {
      imageTopic.unsubscribe();
      ros.close();
      // Si se desmonta el componente, no hace falta poner placeholder aquí
    };
  }, [cameraActive, windowHeight, windowWidth]);

  return (
    <canvas
      ref={canvasRef}
      width={windowWidth}
      height={windowHeight}
      style={{
        width: '100%',
        height: 'auto',
        background: '#222',
        display: 'block',
      }}
    />
  );
};

export default ImageSubscriber;   
