// src/config/dataConfig.ts

export const DATA_CONFIG = {
  // API and data collection settings
  API_POLLING_RATE: 5, // Hz
  BUFFER_TIME_WINDOW: 20, // seconds
  
  // UI refresh rates
  SIMPLE_CHART_REFRESH_RATE: 10, // Hz
  
  // Debug settings
  DEBUG: {
    ENABLE_CONSOLE_LOGS: false, // Set to false to disable most logs
    COMPONENTS: {
      DATA_API: false,
      DATA_BUFFER: false,
      MULTI_VIEW_GRAPH: false,
      SIMPLE_CHART: false,
      RECORDING: true // Keep recording logs enabled
    }
  },
  
  // Recording settings
  MAX_RECORDING_SIZE_MB: 100, // Maximum in-memory size per recording
  MAX_RECORDING_DURATION_SEC: 3600 // 1 hour maximum recording time
};

// Helper for conditional logging
export const debugLog = (component: keyof typeof DATA_CONFIG.DEBUG.COMPONENTS, ...args: any[]) => {
  if (DATA_CONFIG.DEBUG.ENABLE_CONSOLE_LOGS && DATA_CONFIG.DEBUG.COMPONENTS[component]) {
    console.log(`[${component}]`, ...args);
  }
};

// Helper for logging only certain types of messages (less frequent)
export const logSummary = (component: keyof typeof DATA_CONFIG.DEBUG.COMPONENTS, ...args: any[]) => {
  // Always log summaries, regardless of debug settings
  console.log(`[${component}]`, ...args);
};