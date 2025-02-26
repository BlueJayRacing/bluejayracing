// src/pages/DataVisPage.tsx
import React from 'react';
import DataPage from '../components/data_view/DataPage';
import { ThemeProvider, createTheme } from '@mui/material/styles';

// Create a theme instance
const theme = createTheme({
  palette: {
    primary: {
      main: '#3f51b5',
    },
    secondary: {
      main: '#f50057',
    },
  },
  typography: {
    fontFamily: '"Roboto", "Helvetica", "Arial", sans-serif',
  },
});

const DataVisPage: React.FC = () => {
  return (
    <ThemeProvider theme={theme}>
      <DataPage />
    </ThemeProvider>
  );
};

export default DataVisPage;