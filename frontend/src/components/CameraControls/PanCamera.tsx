import React, { useState, useRef, useEffect, FC} from 'react';
import { Canvas, render } from '@react-three/fiber';
import { OrbitControlProps, Orbit} from '../components/CameraControls/Orbit';
import { useThree , useFrame} from '@react-three/fiber';
import { RenderTarget } from 'three';

export const PanCamera: FC<OrbitControlProps> = ({
    horizontalPosition, verticalPosition, 
    setHorizontalPosition, setVerticalPosition, 
    isGoingToFront, setIsGoingToFront,
    isGoingToBack, setIsGoingToBack,
    isGoingToLeft, setIsGoingToLeft,
    isGoingToRight, setIsGoingToRight
  
  }) => {
    // how much to increment the camera by when panning
    const increment = 0.01;
    // the threshold for how close the camera has to be to
    // a certain position (front, back, left, right), before it
    // is considered at that position
    const threshold = 0.01;
    // the front, back, left, and right view have the same verticalPosition
    const goToPresetVerticalPosition = () => {
      if (verticalPosition > 1.5) {
        setVerticalPosition(verticalPosition - increment);
      } else if (verticalPosition < 1.5) {
        setVerticalPosition(verticalPosition + increment);
      }
    }
    useFrame(() => {
      if(isGoingToFront) {
        if (horizontalPosition > -1.55) {
          setHorizontalPosition((horizontalPosition - increment));
        } else if (horizontalPosition < -1.55) {
          setHorizontalPosition(horizontalPosition + increment);
        } 
        goToPresetVerticalPosition();
        if (Math.abs(horizontalPosition + 1.55) < threshold && Math.abs(verticalPosition - 1.5) < threshold) {
          setIsGoingToFront(false);
        }
      }
      if(isGoingToBack) {
        if (horizontalPosition > 1.55) {
          setHorizontalPosition((horizontalPosition - increment));
        } else if (horizontalPosition < 1.55) {
          setHorizontalPosition(horizontalPosition + increment);
        } 
        goToPresetVerticalPosition();
        if (Math.abs(horizontalPosition - 1.55 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
          setIsGoingToBack(false);
        }
      }
      if(isGoingToLeft) {
        if (horizontalPosition > 0) {
          setHorizontalPosition((horizontalPosition - increment));
        } else if (horizontalPosition < 0) {
          setHorizontalPosition(horizontalPosition + increment);
        } 
        goToPresetVerticalPosition();
        if (Math.abs(horizontalPosition - 0 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
          setIsGoingToLeft(false);
        }
      }
      if(isGoingToRight) {
        if (horizontalPosition > 3.14) {
          setHorizontalPosition((horizontalPosition - increment));
        } else if (horizontalPosition < 3.14) {
          setHorizontalPosition(horizontalPosition + increment);
        } 
        goToPresetVerticalPosition();
        if (Math.abs(horizontalPosition - 3.14 ) < threshold && Math.abs(verticalPosition - 1.50) < threshold) {
          setIsGoingToRight(false);
        }
      }
  
    })
  
    return(
      <></>
    )
  }