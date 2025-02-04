import React, { useState, useRef, useEffect, FC} from 'react';
import { Canvas, render } from '@react-three/fiber';
import { CameraControlProps } from 'src/components/CameraControls/CameraControls';
import { useThree , useFrame} from '@react-three/fiber';
import { RenderTarget } from 'three';
import { CameraPositionProps } from './CameraControls';

export interface RequiredPosition {
    requiredHorizontalPosition: number;
    requiredVerticalPosition: number;
}

export interface PanningProps extends CameraControlProps, RequiredPosition{
    isGoingToRequiredPosition: boolean;
    //requiredPosition: RequiredPosition[];
    setRequiredHorizontalPosition: (value: number) => void;
    setRequiredVerticalPosition: (value: number) => void;
    threshold: number;
    setThreshold: (value: number) => void;
    increment: number;
    setIncrement: (value: number) => void;
}

export const PanCamera: FC<PanningProps> = ({
    horizontalPosition, verticalPosition, 
    setHorizontalPosition, setVerticalPosition, 
    requiredHorizontalPosition, requiredVerticalPosition,
    increment, isGoingToRequiredPosition
  }) => {
    useFrame(() => {
        if (isGoingToRequiredPosition) {
            if (horizontalPosition > requiredHorizontalPosition) {
                setHorizontalPosition((horizontalPosition - increment));
            } else if (horizontalPosition < requiredHorizontalPosition) {
                setHorizontalPosition(horizontalPosition + increment);
            } 
            if (verticalPosition > requiredVerticalPosition) {
                setVerticalPosition(verticalPosition - increment);
            } else if (verticalPosition < requiredVerticalPosition) {
                setVerticalPosition(verticalPosition + increment);
            }
        }

    })
    return(
      <></>
    )
  }