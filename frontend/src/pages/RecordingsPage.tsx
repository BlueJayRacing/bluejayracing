// src/pages/RecordingsPage.tsx
import React from 'react';
import { DataProvider } from '../components/shared/DataContext';
import { Typography, Container, Box, Paper } from '@mui/material';
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

const RecordingsPage: React.FC = () => {
  return (
    <ThemeProvider theme={theme}>
      <DataProvider>
        <Container>
          <Box py={4}>
            <Typography variant="h4" component="h1" gutterBottom>
              Recordings
            </Typography>
            
            <Paper elevation={2} sx={{ p: 3, mt: 2 }}>
              <Typography variant="body1">
                The recordings feature will be implemented in a future update.
              </Typography>
            </Paper>
          </Box>
        </Container>
      </DataProvider>
    </ThemeProvider>
  );
};

export default RecordingsPage;