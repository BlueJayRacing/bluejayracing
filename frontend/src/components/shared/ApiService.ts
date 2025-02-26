// src/components/shared/ApiService.ts
import axios from 'axios';
import { DATA_CONFIG, debugLog } from '../../config/dataConfig';

const API_BASE_URL = 'http://localhost:9365'; // Mock data API server port

export const ApiService = {
  getAllChannelData: async (timeWindowMs = DATA_CONFIG.BUFFER_TIME_WINDOW * 1000) => {
    try {
      // For backward compatibility, the server currently uses max_samples
      // We'll add a time-based parameter here for future use
      const response = await axios.get(`${API_BASE_URL}/data/all`, {
        params: { 
          max_samples: 1000,  // This will be removed later
          time_window_ms: timeWindowMs  // New parameter for time-based requests
        }
      });
      
      debugLog('DATA_API', `Fetched all channel data, window: ${timeWindowMs}ms`);
      return response.data;
    } catch (error) {
      console.error('Error fetching all channel data:', error);
      throw error;
    }
  },

  getChannelData: async (channelName: string, timeWindowMs = DATA_CONFIG.BUFFER_TIME_WINDOW * 1000) => {
    try {
      const response = await axios.get(`${API_BASE_URL}/data/${channelName}`, {
        params: { 
          max_samples: 1000,  // This will be removed later
          time_window_ms: timeWindowMs  // New parameter for time-based requests
        }
      });
      
      debugLog('DATA_API', `Fetched data for ${channelName}, window: ${timeWindowMs}ms`);
      return response.data;
    } catch (error) {
      console.error(`Error fetching data for channel ${channelName}:`, error);
      throw error;
    }
  },

  // Method to get channel metadata
  getChannelMetadata: async () => {
    try {
      // When the server supports this endpoint in the future
      const response = await axios.get(`${API_BASE_URL}/metadata`).catch(() => {
        // Fallback to generating metadata locally if endpoint not available
        debugLog('DATA_API', 'Metadata endpoint not available, generating locally');
        return { data: generateDefaultMetadata() };
      });
      
      return response.data;
    } catch (error) {
      console.error('Error fetching channel metadata:', error);
      // Fallback to generated metadata
      return { metadata: generateDefaultMetadata() };
    }
  },

  // Method to determine maximum data rate the API can support
  testMaxDataRate: async () => {
    try {
      const startTime = performance.now();
      const response = await axios.get(`${API_BASE_URL}/data/all`, {
        params: { max_samples: 100 } // Small sample for quick response
      });
      const endTime = performance.now();
      const responseTime = endTime - startTime;
      
      // Calculate theoretical max rate based on response time
      // Add buffer to account for network variability
      const theoreticalMaxHz = 1000 / responseTime;
      const safeMaxHz = Math.floor(theoreticalMaxHz * 0.75); // 75% of theoretical max
      
      debugLog('DATA_API', `Max data rate test: ${safeMaxHz}Hz (response time: ${responseTime.toFixed(2)}ms)`);
      return Math.min(Math.max(1, safeMaxHz), 30); // Between 1Hz and 30Hz
    } catch (error) {
      console.error('Error testing max data rate:', error);
      return 5; // Default to 5Hz if test fails
    }
  }
};

// Helper function to generate default metadata when API doesn't provide it
function generateDefaultMetadata() {
  return {
    channels: [
      {
        name: "linpot_front_left",
        type: 0, // LINEAR_POTENTIOMETER
        sample_rate: 2000,
        transmission_rate: 100,
        location: "FrontLeft",
        units: "V",
        description: "Linear potentiometer measuring front left suspension travel",
        min_value: 0.0,
        max_value: 5.0
      },
      {
        name: "linpot_front_right",
        type: 0, // LINEAR_POTENTIOMETER
        sample_rate: 2000,
        transmission_rate: 100,
        location: "FrontRight",
        units: "V",
        description: "Linear potentiometer measuring front right suspension travel",
        min_value: 0.0,
        max_value: 5.0
      },
      {
        name: "linpot_rear_left",
        type: 0, // LINEAR_POTENTIOMETER
        sample_rate: 2000,
        transmission_rate: 100,
        location: "RearLeft",
        units: "V",
        description: "Linear potentiometer measuring rear left suspension travel",
        min_value: 0.0,
        max_value: 5.0
      },
      {
        name: "linpot_rear_right",
        type: 0, // LINEAR_POTENTIOMETER
        sample_rate: 2000,
        transmission_rate: 100,
        location: "RearRight",
        units: "V",
        description: "Linear potentiometer measuring rear right suspension travel",
        min_value: 0.0,
        max_value: 5.0
      },
      {
        name: "wheel_speed_fl",
        type: 1, // HALL_EFFECT_SPEED
        sample_rate: 2000,
        transmission_rate: 100,
        location: "FrontLeft",
        units: "RPM",
        description: "Hall effect sensor measuring front left wheel speed",
        min_value: 0.0,
        max_value: 10000.0
      },
      {
        name: "wheel_speed_fr",
        type: 1, // HALL_EFFECT_SPEED
        sample_rate: 2000,
        transmission_rate: 100,
        location: "FrontRight",
        units: "RPM",
        description: "Hall effect sensor measuring front right wheel speed",
        min_value: 0.0,
        max_value: 10000.0
      },
      {
        name: "wheel_speed_rl",
        type: 1, // HALL_EFFECT_SPEED
        sample_rate: 2000,
        transmission_rate: 100,
        location: "RearLeft",
        units: "RPM",
        description: "Hall effect sensor measuring rear left wheel speed",
        min_value: 0.0,
        max_value: 10000.0
      },
      {
        name: "wheel_speed_rr",
        type: 1, // HALL_EFFECT_SPEED
        sample_rate: 2000,
        transmission_rate: 100,
        location: "RearRight",
        units: "RPM",
        description: "Hall effect sensor measuring rear right wheel speed",
        min_value: 0.0,
        max_value: 10000.0
      },
      {
        name: "brake_pressure_front",
        type: 2, // BRAKE_PRESSURE
        sample_rate: 2000,
        transmission_rate: 100,
        location: "Front",
        units: "PSI",
        description: "Pressure sensor measuring front brake line pressure",
        min_value: 0.0,
        max_value: 2000.0
      },
      {
        name: "brake_pressure_rear",
        type: 2, // BRAKE_PRESSURE
        sample_rate: 2000,
        transmission_rate: 100,
        location: "Rear",
        units: "PSI",
        description: "Pressure sensor measuring rear brake line pressure",
        min_value: 0.0,
        max_value: 2000.0
      },
      {
        name: "steering_angle",
        type: 4, // STEERING_ENCODER
        sample_rate: 2000,
        transmission_rate: 100,
        location: "Center",
        units: "Degrees",
        description: "Encoder measuring steering wheel angle",
        min_value: -180.0,
        max_value: 180.0
      },
      {
        name: "axle_torque",
        type: 5, // AXLE_TORQUE
        sample_rate: 2000,
        transmission_rate: 100,
        location: "Center",
        units: "Nm",
        description: "Torque sensor measuring drive axle torque",
        min_value: 0.0,
        max_value: 500.0
      }
    ]
  };
}