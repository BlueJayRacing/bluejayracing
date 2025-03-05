// Updated App.tsx with new routes
import React from 'react';
import { BrowserRouter as Router, Routes, Route, Link } from 'react-router-dom';
import { SuspensionProvider } from './contexts/SuspensionContext';
import { DataProvider } from './components/shared/DataContext';
import HomePage from './pages/HomePage';
import Graphing from './pages/Graphing';
import DataVisPage from './pages/DataVisPage';
import RecordingsPage from './pages/RecordingsPage';
import PlaybackPage from './pages/PlaybackPage';
import { AppBar, Toolbar, Typography, Button, Box } from '@mui/material';

const App: React.FC = () => {
  return (
    <DataProvider>
      <SuspensionProvider>
        <Router>
          <AppBar position="static">
            <Toolbar>
              <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
                BlueJay Racing
              </Typography>
              <Button color="inherit" component={Link} to="/">
                Home
              </Button>
              <Button color="inherit" component={Link} to="/data">
                Data
              </Button>
              <Button color="inherit" component={Link} to="/recordings">
                Recordings
              </Button>
              <Button color="inherit" component={Link} to="/graphing">
                Graphing
              </Button>
            </Toolbar>
          </AppBar>

          <Box sx={{ pt: 2 }}>
            <Routes>
              <Route path="/" element={<HomePage />} />
              <Route path="/data" element={<DataVisPage />} />
              <Route path="/recordings" element={<RecordingsPage />} />
              <Route path="/playback/:recordingId" element={<PlaybackPage />} />
              <Route path="/graphing" element={<Graphing />} />
              {/* Add more routes as needed */}
            </Routes>
          </Box>
        </Router>
      </SuspensionProvider>
    </DataProvider>
  );
};

export default App;