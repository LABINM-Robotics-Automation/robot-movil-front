import axios from 'axios';

const backend = axios.create({
  // baseUrl: `http://192.168.0.10:8000${endpoint}`, // Replace with your backend URL
  baseUrl: `http://localhost:8000`,
  // baseUrl: `http://127.0.0.1:8000${endpoint}`,
})

export default backend;
