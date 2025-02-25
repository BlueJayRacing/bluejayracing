// src/utils/generateMockData.ts

// Function to generate mock data that simulates the API response
export const generateMockData = () => {
  const now = Date.now();
  const channels = [
    {
      name: "axle_torque",
      type: 5,
      min_value: 0.0, 
      max_value: 500.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 250 + Math.sin(i * 0.08) * 150
      }))
    },
    {
      name: "brake_pressure_front",
      type: 2,
      min_value: 0.0, 
      max_value: 2000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: i > 80 ? 1500 * Math.random() : 100 * Math.random()
      }))
    },
    {
      name: "brake_pressure_rear",
      type: 2,
      min_value: 0.0, 
      max_value: 2000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: i > 80 ? 1200 * Math.random() : 80 * Math.random()
      }))
    },
    {
      name: "linpot_front_left",
      type: 0,
      min_value: 0.0, 
      max_value: 5.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 2.5 + Math.sin(i * 0.1) * 1.2
      }))
    },
    {
      name: "linpot_front_right",
      type: 0,
      min_value: 0.0, 
      max_value: 5.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 2.7 + Math.sin(i * 0.1 + 1) * 1.1
      }))
    },
    {
      name: "linpot_rear_left",
      type: 0,
      min_value: 0.0, 
      max_value: 5.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 2.3 + Math.sin(i * 0.1 + 2) * 1.0
      }))
    },
    {
      name: "linpot_rear_right",
      type: 0,
      min_value: 0.0, 
      max_value: 5.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 2.6 + Math.sin(i * 0.1 + 3) * 0.9
      }))
    },
    {
      name: "steering_angle",
      type: 4,
      min_value: -180.0, 
      max_value: 180.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 45 * Math.sin(i * 0.03)
      }))
    },
    {
      name: "wheel_speed_fl",
      type: 1,
      min_value: 0.0, 
      max_value: 10000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 3000 + Math.sin(i * 0.05) * 500
      }))
    },
    {
      name: "wheel_speed_fr",
      type: 1,
      min_value: 0.0, 
      max_value: 10000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 3050 + Math.sin(i * 0.05 + 0.5) * 520
      }))
    },
    {
      name: "wheel_speed_rl",
      type: 1,
      min_value: 0.0, 
      max_value: 10000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 2950 + Math.sin(i * 0.05 + 1) * 480
      }))
    },
    {
      name: "wheel_speed_rr",
      type: 1,
      min_value: 0.0, 
      max_value: 10000.0,
      samples: Array.from({ length: 100 }, (_, i) => ({
        timestamp: now - (99 - i) * 100,
        value: 3020 + Math.sin(i * 0.05 + 1.5) * 490
      }))
    },
  ];
  
  return { channels };
};