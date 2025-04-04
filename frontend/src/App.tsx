// src/App.tsx

import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { ThemeProvider, createTheme } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { DataProvider } from './components/shared/DataContext';
import { TimestampProvider } from './contexts/TimestampContext';
import DataPage from './components/data_view/DataPage';
import DataVisPage from './pages/DataVisPage';

// Import our custom styles
import './styles/components.css';

// Create a theme instance
const theme = createTheme({
  palette: {
    primary: {
      main: '#3b82f6',
    },
    secondary: {
      main: '#6366f1',
    },
    error: {
      main: '#ef4444',
    },
    background: {
      default: '#f1f5f9',
    },
  },
  typography: {
    fontFamily: [
      'Inter',
      '-apple-system',
      'BlinkMacSystemFont',
      '"Segoe UI"',
      'Roboto',
      '"Helvetica Neue"',
      'Arial',
      'sans-serif',
    ].join(','),
  },
  components: {
    MuiButton: {
      styleOverrides: {
        root: {
          textTransform: 'none',
          borderRadius: '0.375rem',
        },
      },
    },
    MuiPaper: {
      styleOverrides: {
        root: {
          borderRadius: '0.375rem',
        },
      },
    },
  },
});

const App: React.FC = () => {
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <TimestampProvider>
      <DataProvider>
        <Router>
          <Routes>
            <Route path="/" element={<DataPage />} />
            <Route path="/data" element={<DataPage />} />
            <Route path="/visualize" element={<DataVisPage />} />
          </Routes>
        </Router>
      </DataProvider>
      </TimestampProvider>
    </ThemeProvider>
  );
};

export default App;