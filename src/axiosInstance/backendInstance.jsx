import axios from 'axios';

const backend = {
  // ip: 'localhost',
  ip : '192.168.0.10',
  port: '8000',
  get url() {
    return `http://${this.ip}:${this.port}`;
  },
  get axios() {
    return axios.create({ baseURL: this.url });
  },
  buildUrl(path) {
    return `${this.url}${path}`;
  },
};


export default backend;
