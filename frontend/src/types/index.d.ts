// Define any global types or interfaces here

// For example, if you have a type for mapping data
export interface MappingDataEntry {
    shockExtension: number;
    controlArmAngle: number;
  }
  
  declare module '*.glb' {
    const src: string;
    export default src;
  }
  
  declare module '*.json' {
    const value: any;
    export default value;
  }
  