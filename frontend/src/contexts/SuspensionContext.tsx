// SuspensionContext.tsx
import React, { createContext, useState, useEffect } from 'react';

export interface SuspensionState {
  shockExtension: number;
  showSuspension: boolean;
  suspensionAngle: number;
  leftShockExtension: number;
  rightShockExtension: number;
  leftFrontShockExtension: number;
  leftBackShockExtension: number;
  rightFrontShockExtension: number;
  rightBackShockExtension: number;

}

interface SuspensionContextProps extends SuspensionState {
  // setShockExtension: (value: number) => void;
  setShowSuspension: (value: boolean) => void;
  // suspensionAngle: (value: number) => void;
  // setLeftShockExtension: (value: number) => void;
  // setRightShockExtension: (value: number) => void;

  setLeftFrontShockExtension: (value: number) => void;
  setLeftBackShockExtension: (value: number) => void;
  setRightFrontShockExtension: (value: number) => void;
  setRightBackShockExtension: (value: number) => void;
}

export const SuspensionContext = createContext<SuspensionContextProps | null>(null);

export const SuspensionProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  // const [shockExtension, setShockExtension] = useState<number>(0);
  const [showSuspension, setShowSuspension] = useState<boolean>(true);
  // const [suspensionAngle, setSuspensionAngle] = useState<number>(0);
  // const [leftShockExtension, setLeftShockExtension] = useState<number>(0);
  // const [rightShockExtension, setRightShockExtension] = useState<number>(0);

  const [leftFrontShockExtension, setLeftFrontShockExtension] = useState<number>(0);
  const [leftBackShockExtension, setLeftBackShockExtension] = useState<number>(0);
  const [rightFrontShockExtension, setRightFrontShockExtension] = useState<number>(0);
  const [rightBackShockExtension, setRightBackShockExtension] = useState<number>(0);

  return (
    <SuspensionContext.Provider
      value={{
        // shockExtension,
        showSuspension,
        // setShockExtension,
        setShowSuspension,
        // suspensionAngle,
        // setSuspensionAngle,
        // leftShockExtension,
        // setLeftShockExtension,
        // rightShockExtension,
        // setRightShockExtension,

        leftFrontShockExtension,
        setLeftFrontShockExtension,
        leftBackShockExtension,
        setLeftBackShockExtension,
        rightFrontShockExtension,
        setRightFrontShockExtension,
        rightBackShockExtension,
        setRightBackShockExtension,
      }}
    >
      {children}
    </SuspensionContext.Provider>
  );
};