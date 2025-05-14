// src/hooks/useDataContext.ts

import { useContext } from 'react';
import { DataContext, DataContextType } from '../components/shared/DataContext';

// Hook to access the data context
export const useDataContext = (): DataContextType => {
  const context = useContext(DataContext);
  
  if (!context) {
    throw new Error('useDataContext must be used within a DataProvider');
  }
  
  return context;
};