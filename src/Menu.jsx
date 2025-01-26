import React, { useState, useEffect } from 'react';
import axios from 'axios';
import ImageSubscriber from './ImageSubscriber';
import TopicIndicator from './TopicIndicator';
import RecordingIndicator from './RecordingIndicator'
import RecordedFilesList from './RecordedFileList'
import backend from './axiosInstance/backendInstance'


function Menu() {

    const [cameraActive, setCameraActive] = useState(false) 
    const [windowHeight, setWindowHeight] = useState(window.innerHeight)
    const [windowWidth,  setWindowWidth] = useState(window.innerWidth)
    const [activeMenu, setActiveMenu] = useState('camera'); // Track active menu ('camera' or 'files')

    useEffect(() => {
        const handleResize = () => {
            setWindowHeight(window.innerHeight);
            setWindowWidth(window.innerWidth)
        };

        window.addEventListener('resize', handleResize);

        return () => {window.removeEventListener('resize', handleResize)};
    }, []);


    const handleRequest = async (endpoint, method = 'GET') => {
        try {
            const response = await backend.axios({
                method: method,
                url: `${endpoint}`,
                data: method !== 'GET' ? { key: 'value' } : null,
            });
            
            if (endpoint == '/stop_camera'){
                setTimeout(() => {window.location.reload()}, 200);
            }
            console.log('Response:', response.data);

        } catch (error) {
            console.error('Error:', error);
        }
    };

    const startCameraSequence = async () => {
        try {
            // Show loading state if needed
            console.log('Iniciando secuencia de requests...');
            
            // Execute requests in sequence
            await handleRequest('/start_camera', 'POST');
            console.log('Cámara iniciada');
            
            // Wait a bit for camera to initialize
            await new Promise(resolve => setTimeout(resolve, 1000));
            
            await handleRequest('/start_image_processor', 'POST');
            console.log('Procesador iniciado');
            
            await new Promise(resolve => setTimeout(resolve, 500));
            
            await handleRequest('/start_websocket', 'POST');
            console.log('Websocket iniciado');
            
            console.log('Secuencia completada exitosamente');
        } catch (error) {
            console.error('Error en la secuencia:', error);
        }
    }

    const stopCameraSequence = async () => {
        try {
            // Show loading state if needed
            console.log('Iniciando secuencia de requests...');
            
            // Execute requests in sequence
            await handleRequest('/stop_camera', 'POST');
            console.log('Cámara detenida');
            
            // Wait a bit for camera to initialize
            await new Promise(resolve => setTimeout(resolve, 1000));
            
            await handleRequest('/stop_image_processor', 'POST');
            console.log('Procesador detenido');
            
            await new Promise(resolve => setTimeout(resolve, 500));
            
            await handleRequest('/stop_websocket', 'POST');
            console.log('Websocket detenido');
            
            console.log('Secuencia completada exitosamente');
        } catch (error) {
            console.error('Error en la secuencia:', error);
        }
    }

    const toggleMenu = (menu) => setActiveMenu(menu);

    return (
    <div style={{ ...styles.container, height: windowHeight * 0.8 }}>
        {activeMenu === 'camera' && (
        <div style={{ ...styles.container, height: windowHeight * 0.8 }} >
            <div style={styles.imageContainer}>
            <ImageSubscriber cameraActive={true} windowHeight={windowHeight} windowWidth={windowWidth} />
            </div>
            <div style={styles.controlsContainer}>

            <button style={styles.button} onClick={() => toggleMenu('files')}>
                Menú archivos guardados
            </button>

            <button style={styles.button} onClick={startCameraSequence}>
                Iniciar cámara
            </button>

            <button style={styles.button} onClick={stopCameraSequence}>
                Detener cámara
            </button>

            <button style={styles.button} onClick={() => handleRequest('/start_record', 'POST')}>
                Iniciar grabación
            </button>
            <button style={styles.button} onClick={() => handleRequest('/stop_record', 'POST')}>
                Detener grabación
            </button>

            <TopicIndicator style={styles.indicator} />
            <RecordingIndicator style={styles.indicator} />
            </div>
        </div>
        )}
        {activeMenu === 'files' && (
        <div style={{ ...styles.container, height: 200}} >
            <div style={{ width: 1100}}>
            <RecordedFilesList windowWidth={1100}/>
            </div>
            <div style={styles.controlsContainer}>
            <button style={styles.button} onClick={() => toggleMenu('camera')}>
                Menú cámara Zed
            </button>
            </div>
        </div>)
        }
    </div>
    ); 
}

const styles = {
  container: {
    display: 'flex',
    color: '#ddd',
    fontFamily: 'Arial, sans-serif',
  },
  imageContainer: {
    flex: 10,  // Takes 3/4 of the width
    backgroundColor: '#111',  // Optional: background color for better contrast
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  },
  controlsContainer: {
    flex: 2,
    'padding-top' : '0px',
    'padding-left': '20px',
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'flex-start',
  },
  button : {
    width: '200px',
    'margin-bottom' : '20px',
    'border-radius' : '1px',

  },
  indicator : {
    width: '200px',
    'border-radius': '8px',
    // 'border': '1px solid transparent',
    // 'padding': '0.6em 1.2em',
    'font-size': '1em',
    'font-weight': '500',
    'font-family': 'inherit',
    'background-color': '#111',
    'cursor': 'pointer',
    'transition': 'border-color 0.25s',
    // 'padding' : '10px',
    'textAlign': 'center', 
    'marginTop': '10px',
    'background': '#111',
    'border-radius' : '1px',
    'margin-bottom' : '20px',
    'border-radius' : '1px'
  } 
}

export default Menu;

