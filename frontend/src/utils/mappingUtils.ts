// Assume mappingData is an array of objects with shockExtension and controlArmAngle
// import mappingData from './assets/mappingData.json';

export function getControlArmAngle(shockExtension: number): number {
//   // Find the two data points between which the shockExtension falls
//   const lower = mappingData.reduce((prev, curr) =>
//     curr.shockExtension <= shockExtension ? curr : prev
//   );
//   const upper = mappingData.find(
//     (data) => data.shockExtension >= shockExtension
//   ) || lower;

//   // Perform linear interpolation
//   if (lower.shockExtension === upper.shockExtension) {
//     return lower.controlArmAngle;
//   }

//   const ratio =
//     (shockExtension - lower.shockExtension) /
//     (upper.shockExtension - lower.shockExtension);

//   const angle =
//     lower.controlArmAngle +
//     ratio * (upper.controlArmAngle - lower.controlArmAngle);

//   return angle;
  return NaN;
}
