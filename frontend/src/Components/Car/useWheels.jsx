import { useCompoundBody } from "@react-three/cannon";
import { useRef } from "react";

export const useWheels = (width, height, front, radius) => {
  const wheels = [useRef(null), useRef(null), useRef(null), useRef(null)];

  const wheelInfo = {
    radius,
    directionLocal: [0, -1, 0],
    axleLocal: [1, 0, 0],
    suspensionStiffness: 30,
    suspensionRestLength: 0.1,
    frictionSlip: 5,
    dampingRelaxation: 1.3,
    dampingCompression: 2.4,
    // dampingRelaxation: 2.3,
    // dampingCompression: 4.4,
    maxSuspensionForce: 100000,
    rollInfluence: 0.05,
    maxSuspensionTravel: 10,
    customSlidingRotationalSpeed: -30,
    useCustomSlidingRotationalSpeed: true,
  };

  const wheelInfos = [
    {
      ...wheelInfo,
      chassisConnectionPointLocal: [-width * 0.65, height * 0.4, front],
      isFrontWheel: true,
    },
    {
      ...wheelInfo,
      chassisConnectionPointLocal: [width * 0.65, height * 0.4, front],
      isFrontWheel: true,
    },
    {
      ...wheelInfo,
      chassisConnectionPointLocal: [-width * 0.65, height * 0.4, -front],
      isFrontWheel: false,
    },
    {
      ...wheelInfo,
      chassisConnectionPointLocal: [width * 0.65, height * 0.4, -front],
      isFrontWheel: false,
    },
  ];

  const propsFunc = () => ({
    collisionFilterGroup: 0,
    mass: 10,
    shapes: [
      {
        args: [wheelInfo.radius, wheelInfo.radius, 0.015, 16],
        rotation: [0, 0, -Math.PI / 2],
        type: "Cylinder",
      },
    ],
    type: "Kinematic",
  });

  const [w1, w1r] = (useCompoundBody(propsFunc, wheels[0]));
  const [w2, w2r] = useCompoundBody(propsFunc, wheels[1]);
  const [w3, w3r] = useCompoundBody(propsFunc, wheels[2]);
  const [w4, w4r] = useCompoundBody(propsFunc, wheels[3]);

  return [wheels, wheelInfos, [w1r, w2r, w3r, w4r]];
};