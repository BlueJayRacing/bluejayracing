import * as React from 'react';
import PropTypes from 'prop-types';
import Tabs from '@mui/material/Tabs';
import Tab from '@mui/material/Tab';
import Box from '@mui/material/Box';
import { createTheme, ThemeProvider, CssBaseline } from '@mui/material';
import { BrowserRouter as Router, Routes, Route, Link, useLocation } from 'react-router-dom';
import PlaybackRateMenuButtonExmaple from "./Pages/Player";
import Map from "./Pages/Map.jsx";
import World from "./Pages/World.jsx" 
import Table from './Pages/Table';

const theme = createTheme({
  typography: {
    fontFamily: 'Quadon',
  },
  components: {
    MuiCssBaseline: {
      styleOverrides: `
      @font-face {
        font-family: 'Quadon';
        src: url('/fonts/Quadon.ttf') format('truetype');
      }
      `,
    },
  },
});

function TabsComponent() {
  const location = useLocation();
  const currentPath = location.pathname;
  const tabNames = ["/table", "/world", "/map", "/video-playback"];
  const value = tabNames.indexOf(currentPath);

  return (
    <Box sx={{ width: '100%' }}>
      <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
        <Tabs value={value} centered aria-label="basic tabs example">
          <Tab label="Table" component={Link} to="/table" />
          <Tab label="World" component={Link} to="/world" />
          <Tab label="Map" component={Link} to="/map" />
          <Tab label="Video Playback" component={Link} to="/video-playback" />
        </Tabs>
      </Box>
    </Box>
  );
}

function BasicTabs() {
  return (
    <Router>
      <ThemeProvider theme={theme}>
        <CssBaseline />
        <TabsComponent />
        <Routes>
          <Route path="/table" element={<Table />} />
          <Route path="/world" element={<World />} />
          <Route path="/map" element={<Map position={[39.32845515755447, -76.62242889404298]} />} />
          <Route path="/video-playback" element={<PlaybackRateMenuButtonExmaple />} />
        </Routes>
      </ThemeProvider>
    </Router>
  );
}

export default BasicTabs;