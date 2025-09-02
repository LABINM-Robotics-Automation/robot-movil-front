import React, { useState, useEffect } from 'react';
import backend from './axiosInstance/backendInstance'
import axios from 'axios';

const FileList = (
  windowWidth = 900
) => {
  const [files, setFiles] = useState([]);
  const [mp4Files, setMp4Files] = useState([]);
  const [loading, setLoading] = useState(true);
  const [mp4Loading, setMp4Loading] = useState(false);
  const [convertingFiles, setConvertingFiles] = useState(new Set());

  useEffect(() => {
    fetchFiles();
    fetchMp4Files();
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

  const fetchMp4Files = async () => {
    try {
      setMp4Loading(true);
      const { data } = await backend.axios.get('/list_mp4_files');
      setMp4Files(data.files || []);
    } catch (error) {
      console.error('Error fetching MP4 files:', error);
      setMp4Files([]);
    } finally {
      setMp4Loading(false);
    }
  };

  const handleDownload = (fileName) => {
    const fileUrl = backend.buildUrl(`/download/${fileName}`);
    window.location.href = fileUrl;
  };

  const handleDownloadMp4 = (fileName) => {
    const fileUrl = backend.buildUrl(`/download_mp4/${fileName}`);
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

  const handleDeleteMp4 = async (fileName) => {
    try {
      const { status, data } = await backend.axios.delete(`/delete_mp4/${fileName}`);

      if (status === 200) {
        setMp4Files((prevFiles) => prevFiles.filter((file) => file !== fileName));
        alert('MP4 file deleted successfully');
      } else {
        alert(data?.error || 'Failed to delete the MP4 file');
      }
    } catch (error) {
      console.error('Error deleting MP4 file:', error);
      alert('Failed to delete the MP4 file');
    }
  };

  const handleConvertToMp4 = async (fileName) => {
    if (!fileName.endsWith('.bag')) {
      alert('Solo se pueden convertir archivos .bag');
      return;
    }

    try {
      setConvertingFiles(prev => new Set(prev).add(fileName));
      const { status, data } = await backend.axios.post(`/convert_to_mp4/${fileName}`);

      if (status === 200) {
        alert('Conversión iniciada. El archivo MP4 estará disponible en unos momentos.');
        // Refresh MP4 files list after a delay
        setTimeout(() => {
          fetchMp4Files();
        }, 3000);
      } else {
        alert(data?.error || 'Error al iniciar la conversión');
      }
    } catch (error) {
      console.error('Error converting file:', error);
      alert('Error al convertir el archivo');
    } finally {
      setConvertingFiles(prev => {
        const newSet = new Set(prev);
        newSet.delete(fileName);
        return newSet;
      });
    }
  };
  

  return(
    <div style={{ ...styles.container }}>
      {/* Bag Files Section */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Archivos .bag</h3>
        {loading ? (
          <p>Loading files...</p>
        ) : (
          <ul style={{ ...styles.fileList }}>
            {files.map((file) => (
              <li key={file} style={styles.fileItem}>
                <span style={styles.fileName}>{file}</span>
                <div style={styles.buttonGroup}>
                  <button style={styles.button} onClick={() => handleDelete(file)}>
                    Delete
                  </button>
                  <button style={styles.button} onClick={() => handleDownload(file)}>
                    Download
                  </button>
                  {file.endsWith('.bag') && (
                    <button
                      style={{
                        ...styles.button,
                        ...styles.convertButton,
                        ...(convertingFiles.has(file) ? styles.convertingButton : {})
                      }}
                      onClick={() => handleConvertToMp4(file)}
                      disabled={convertingFiles.has(file)}
                    >
                      {convertingFiles.has(file) ? 'Convirtiendo...' : 'Convertir a MP4'}
                    </button>
                  )}
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>

      {/* MP4 Files Section */}
      <div style={styles.section}>
        <h3 style={styles.sectionTitle}>Archivos MP4 Convertidos</h3>
        <button style={styles.refreshButton} onClick={fetchMp4Files} disabled={mp4Loading}>
          {mp4Loading ? 'Cargando...' : 'Actualizar lista'}
        </button>
        {mp4Loading ? (
          <p>Loading MP4 files...</p>
        ) : mp4Files.length === 0 ? (
          <p>No hay archivos MP4 convertidos</p>
        ) : (
          <ul style={{ ...styles.fileList }}>
            {mp4Files.map((file) => (
              <li key={file} style={styles.fileItem}>
                <span style={styles.fileName}>{file}</span>
                <div style={styles.buttonGroup}>
                  <button style={styles.button} onClick={() => handleDeleteMp4(file)}>
                    Delete
                  </button>
                  <button style={styles.button} onClick={() => handleDownloadMp4(file)}>
                    Download
                  </button>
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>)
};

const styles = {
  container: {
    textAlign: 'center',
    marginTop: '10px',
    marginLeft: 'auto',
    marginRight: 'auto',
    width: '80vw', // Increased width to accommodate more content
    boxSizing: 'border-box',
  },
  section: {
    marginBottom: '30px',
    padding: '20px',
    border: '1px solid #ddd',
    borderRadius: '8px',
    backgroundColor: '#fafafa',
  },
  sectionTitle: {
    margin: '0 0 20px 0',
    color: '#333',
    fontSize: '18px',
    fontWeight: 'bold',
  },
  fileList: {
    listStyleType: 'none',
    padding: 0,
  },
  fileItem: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '15px',
    margin: '10px 0',
    borderRadius: '8px',
    border: '1px solid #ccc',
    backgroundColor: '#fff',
    boxShadow: '0px 2px 4px rgba(0, 0, 0, 0.1)',
    gap: '15px',
  },
  fileName: {
    flex: 1,
    fontSize: '16px',
    color: '#333',
    wordBreak: 'break-all',
  },
  buttonGroup: {
    display: 'flex',
    gap: '10px',
    flexWrap: 'wrap',
  },
  button: {
    padding: '8px 12px',
    fontSize: '14px',
    backgroundColor: '#111',
    color: '#fff',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    transition: 'background-color 0.3s',
    whiteSpace: 'nowrap',
  },
  convertButton: {
    backgroundColor: '#007bff',
  },
  convertingButton: {
    backgroundColor: '#ffc107',
    cursor: 'not-allowed',
  },
  refreshButton: {
    padding: '10px 20px',
    fontSize: '16px',
    backgroundColor: '#28a745',
    color: '#fff',
    border: 'none',
    borderRadius: '4px',
    cursor: 'pointer',
    marginBottom: '15px',
  },
};

export default FileList;

