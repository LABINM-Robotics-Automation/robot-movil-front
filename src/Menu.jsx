import React, { useState } from 'react';
import axios from 'axios';
import ImageSubscriber from './ImageSubscriber';
import TopicIndicator from './TopicIndicator';

function Menu() {
  const [cameraActive, setCameraActive] = useState(false); 

  const handleRequest = async (endpoint, method = 'GET') => {

    try {
      const response = await axios({
        method: method,
        url: `http://localhost:8000${endpoint}`, // Replace with your backend URL
        data: method !== 'GET' ? { key: 'value' } : null, // Include request body if needed
      });

      if (endpoint == '/stop_camera'){
        setTimeout(() => {
          window.location.reload();
        }, 200);
      }
      console.log('Response:', response.data);

    } catch (error) {
      console.error('Error:', error);
    }
  };

  return (
    <div style={{ textAlign: 'center', marginTop: '10px' }}>
      <h2>Cámara Zed2i</h2>
      <TopicIndicator />
      <ImageSubscriber cameraActive={true}/>
      <button onClick={() => handleRequest('/start_camera', 'POST')}> Iniciar cámara </button>
      <button onClick={() => handleRequest('/stop_camera' , 'POST')}> Detener cámara </button>
      <button onClick={() => handleRequest('/start_record', 'POST')}> Iniciar grabación </button>
      <button onClick={() => handleRequest('/stop_record',  'POST')}> Detener grabación </button>
    </div>
  );
}

export default Menu;

