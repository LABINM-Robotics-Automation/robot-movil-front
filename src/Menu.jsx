import React, { useState } from 'react';
import axios from 'axios';
import ImageSubscriber from './ImageSubscriber'; 

function Menu() {

  const [imageSrc, setImageSrc] = useState('');

  const handleRequest = async (endpoint, method = 'GET') => {
    try {
      const response = await axios({
        method: method,
        url: `http://localhost:8000${endpoint}`, // Replace with your backend URL
        data: method !== 'GET' ? { key: 'value' } : null, // Include request body if needed
      });
      console.log('Response:', response.data);

    } catch (error) {
      console.error('Error:', error);
    }
  };


  return (
    <div style={{ textAlign: 'center', marginTop: '10px' }}>
      <h2>Cámara Zed2i</h2>
      <button onClick={() => handleRequest('/start_camera', 'POST')}> Iniciar cámara </button>
      <button onClick={() => handleRequest('/stop_camera' , 'POST')}> Detener cámara </button>
      <button onClick={() => handleRequest('/start_record', 'POST')}> Iniciar grabación </button>
      <button onClick={() => handleRequest('/stop_record',  'POST')}> Detener grabación </button>

    <ImageSubscriber />

    </div>
  );
}

export default Menu;

