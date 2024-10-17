import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import { SuspensionProvider } from './contexts/SuspensionContext';
import HomePage from './pages/HomePage';

const App: React.FC = () => {
  return (
    <SuspensionProvider>
      <Router>
        <Routes>
          <Route path="/" element={<HomePage />} />
          {/* Add more routes as needed */}
        </Routes>
      </Router>
    </SuspensionProvider>
  );
};

export default App;
