// SuspensionContext.tsx
import React, { createContext, useState, useEffect } from 'react';

export interface SuspensionState {
  shockExtension: number;
  showSuspension: boolean;
}

interface SuspensionContextProps extends SuspensionState {
  setShockExtension: (value: number) => void;
  setShowSuspension: (value: boolean) => void;
}

export const SuspensionContext = createContext<SuspensionContextProps | null>(null);

export const SuspensionProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [shockExtension, setShockExtension] = useState<number>(0);
  const [showSuspension, setShowSuspension] = useState<boolean>(true);

  return (
    <SuspensionContext.Provider
      value={{
        shockExtension,
        showSuspension,
        setShockExtension,
        setShowSuspension,
      }}
    >
      {children}
    </SuspensionContext.Provider>
  );
};