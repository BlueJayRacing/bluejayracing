import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { SuspensionProvider } from './contexts/SuspensionContext';
import HomePage from './pages/HomePage';
import Graphing from './pages/Graphing';

const App: React.FC = () => {
  return (
    <SuspensionProvider>
      <Router>
        <Routes>
          <Route path="/" element={<HomePage />} />
          <Route path="/Graphing" element={<Graphing />} />
          {/* Add more routes as needed */}
        </Routes>
      </Router>
    </SuspensionProvider>
  );
};

export default App;
