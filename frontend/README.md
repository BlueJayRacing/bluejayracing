
enable
chrome://flags/#ignore-gpu-blocklist



Root Directory
package.json: Manages project dependencies and scripts.
pnpm-lock.yaml: Lockfile for pnpm.
tsconfig.json: TypeScript compiler configuration.
vite.config.ts: Configuration for Vite.
tailwind.config.js: Configuration for Tailwind CSS.
.eslintrc.js: ESLint configuration for linting rules.
.prettierrc: Prettier configuration for code formatting.
public/
index.html: The main HTML file. Vite uses this as the entry point.
src/
This is where all the application code resides.

assets/
models/: Contains the .glb 3D model files for the vehicle and its components.
components/
Contains reusable React components.

CarModel/CarModel.tsx:

Purpose: Renders the 3D model of the car, applying transformations to the vehicle and its parts based on suspension data.
Contents:
Imports necessary Three.js and react-three-fiber modules.
Uses useGLTF from @react-three/drei to load models.
Applies transformations recursively to components.
Handles rigging logic using mapping functions from utils/mappingUtils.ts.
Logic Handling:
Calculates positions and rotations of parts based on suspension parameters.
Uses helper functions for 3D transformations (e.g., matrices, quaternions).
Listens to context changes from SuspensionContext to update the model.
SuspensionControls/SuspensionControls.tsx:

Purpose: Provides UI controls (sliders) to adjust suspension parameters.
Contents:
Uses MUI components like Slider.
Updates suspension parameters in SuspensionContext.
Validates input ranges (max/min extension and compression).
Logic Handling:
On slider change, calls context methods to update the state.
May debounce inputs to prevent excessive updates.
CameraControls/CameraControls.tsx:

Purpose: Allows users to adjust the camera position and zoom.
Contents:
Uses useThree and useFrame from react-three-fiber to control the camera.
Provides UI controls via MUI or custom components.
Logic Handling:
Updates camera properties (position, rotation, zoom) based on user input.
Ensures camera movements are smooth and constrained within desired ranges.
LightingEffects/LightingEffects.tsx:

Purpose: Manages lighting in the scene, including dynamic and glow effects.
Contents:
Sets up Three.js light sources (e.g., AmbientLight, PointLight).
Applies emissive materials to components for glow effects.
Provides controls to toggle effects.
Logic Handling:
Adjusts light intensities and colors based on user settings.
Updates materials of components to reflect lighting changes.
UI/

Purpose: Contains reusable UI components (buttons, panels, etc.) that may be used across multiple parts of the application.
Contents: Custom styled components using MUI and Tailwind CSS.
contexts/
SuspensionContext.tsx:

Purpose: Provides a global state for suspension parameters.
Contents:
Creates a React context with default suspension values.
Provides a provider component to wrap around the application or specific components.
Includes methods to update suspension parameters.
Logic Handling:
Manages the state for shock extension, control arm angles, etc.
Notifies subscribed components when the state changes.
hooks/
useSuspensionData.ts:

Purpose: Custom hook to poll data from the local API at 5Hz.
Contents:
Uses useEffect to set up an interval for polling.
Fetches data from the local API endpoint.
Updates SuspensionContext with new data.
Logic Handling:
Ensures data synchronization without memory leaks.
Handles errors and retries if the API is unavailable.
use3DTransforms.ts:

Purpose: Provides functions to calculate 3D transformations based on suspension parameters.
Contents:
Contains functions for position, rotation, scaling calculations.
May use math libraries like three or gl-matrix for matrix and vector operations.
Logic Handling:
Centralizes complex math operations.
Reuses transformation logic across components.
utils/
mathUtils.ts:

Purpose: Helper functions for mathematical calculations.
Contents:
Functions for interpolation, mapping, vector operations.
Any custom math functions needed for the suspension mechanics.
Logic Handling:
Provides reusable math operations to avoid code duplication.
mappingUtils.ts:

Purpose: Functions to map suspension parameters to component transformations.
Contents:
Uses the provided mapping file (lookup table) to correlate shock extension with movements.
Functions to interpolate between data points in the mapping.
Logic Handling:
Handles non-linear mappings and ensures smooth transitions.
Applies constraints based on maximum extension/compression.
pages/
HomePage.tsx:

Purpose: The main page that assembles all components.
Contents:
Imports and uses CarModel, SuspensionControls, CameraControls, LightingEffects.
Sets up the layout and styling.
Logic Handling:
Organizes the components in the UI.
May include routing logic if using react-router-dom.
styles/
tailwind.css: Custom Tailwind CSS configurations and base styles.
globals.css: Global CSS resets and styles.
types/
index.d.ts: TypeScript type definitions and interfaces for the project.
Defines types for suspension parameters, model data, context values, etc.
App.tsx
Purpose: The root component that wraps the application.
Contents:
Wraps the application in SuspensionContext.Provider.
Includes routing if using react-router-dom.
Logic Handling:
Sets up the overall application structure.
May include error boundaries or global providers.
main.tsx
Purpose: Entry point for the React application.
Contents:
Renders <App /> into the DOM.
Includes Vite-specific configurations.
vite-env.d.ts
Purpose: Provides TypeScript typings for Vite's environment variables.
Contents:
Type declarations for import.meta.env and other Vite-specific features.
Logic Handling and 3D Transformations
Transformation Logic
Centralized in use3DTransforms.ts and mappingUtils.ts:
These files contain the mathematical logic for transforming the 3D models based on suspension data.
They ensure that calculations are consistent and can be reused across components.
Applying Transformations in CarModel.tsx
Loading Models:

Use useGLTF to load .glb models.
Extract meshes and groups for individual components (wheels, arms, shafts).
Rigging Without Animations:

Apply position and rotation transformations to each component based on the suspension parameters.
Use React's state and effects to update transformations when suspension data changes.
Example Transformation Application:

typescript
Copy code
// Inside CarModel.tsx
const { suspensionData } = useContext(SuspensionContext);
const transform = use3DTransforms(suspensionData);

return (
  <group>
    {/* Car Frame */}
    <mesh geometry={frameGeometry}>
      {/* Apply frame-specific transformations if any */}
    </mesh>

    {/* Wheel */}
    <mesh geometry={wheelGeometry} position={transform.wheelPosition}>
      {/* Apply wheel-specific transformations */}
    </mesh>

    {/* Control Arms */}
    <mesh geometry={controlArmGeometry} rotation={transform.controlArmRotation}>
      {/* Apply control arm-specific transformations */}
    </mesh>

    {/* Shafts */}
    <mesh geometry={shaftGeometry} position={transform.shaftPosition}>
      {/* Apply shaft-specific transformations */}
    </mesh>
  </group>
);
Helper Functions
mathUtils.ts: