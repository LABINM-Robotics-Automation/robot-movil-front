import React, { useState, useEffect } from 'react';
import backend from './axiosInstance/backendInstance'
import axios from 'axios';

const FileList = (
  windowWidth = 900
) => {
  const [files, setFiles] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    fetchFiles();
  }, []);

  const fetchFiles = async () => {
    try {
      setLoading(true);
      const { data } = await backend.axios.get('/list_files');
      setFiles(data.files);
    } catch (error) {
      console.error('Error fetching files:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleDownload = (fileName) => {
    const fileUrl = backend.buildUrl(`/download/${fileName}`);
    window.location.href = fileUrl;
  };

  const handleDelete = async (fileName) => {
    try {
      const { status, data } = await backend.axios.delete(`/delete/${fileName}`);

      if (status === 200) {
        setFiles((prevFiles) => prevFiles.filter((file) => file !== fileName));
        alert('File deleted successfully');
      } else {
        alert(data?.error || 'Failed to delete the file');
      }
    } catch (error) {
      console.error('Error deleting file:', error);
      alert('Failed to delete the file');
    }
  };
  

  return(
    <div style={{ ...styles.container }}>
      {loading ? (
        <p>Loading files...</p>
      ) : (
        <ul style={{ ...styles.fileList }}>
          {files.map((file) => (
            <li key={file} style={styles.fileItem}>
              <span style={styles.fileName}>{file}</span>
              <button style={styles.button} onClick={() => handleDelete(file)}>
                Delete
              </button>
              <button style={styles.button} onClick={() => handleDownload(file)}>
                Download
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>)
};

const styles = {
  container: {
    textAlign: 'center',
    marginTop: '10px',
    marginLeft: 'auto',
    marginRight: 'auto',
    marginLeft: 'auto',
    marginRight: 'auto',
    width: '50vw', // Width is now 80% of the viewport width
    boxSizing: 'border-box', // Ensure padding and border are included in the width
  },
  fileList: {
    listStyleType: 'none',
    padding: 0,
  },
  fileItem: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '10px',
    margin: '10px 0',
    borderRadius: '8px',
    border: '1px solid #ccc',
    backgroundColor: '#f9f9f9',
    boxShadow: '0px 2px 4px rgba(0, 0, 0, 0.1)',
    gap: '10px',
  },
  fileName: {
    flex: 1,
    fontSize: '16px',
    color: '#333',
  },
  button: {
    width: '130px', 
    padding: '8px 12px',
    fontSize: '14px',
    backgroundColor: '#111',
    color: '#fff',
    border: 'none',
    borderRadius: '1px',
    cursor: 'pointer',
    transition: 'background-color 0.3s',
  },
};

export default FileList;

