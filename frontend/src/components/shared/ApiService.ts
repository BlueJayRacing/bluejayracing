// src/components/shared/ApiService.ts
import axios from 'axios';

const API_BASE_URL = 'http://localhost:9365'; // Mock data API server port

export const ApiService = {
  getAllChannelData: async (maxSamples = 1000) => {
    try {
      const response = await axios.get(`${API_BASE_URL}/data/all`, {
        params: { max_samples: maxSamples }
      });
      return response.data;
    } catch (error) {
      console.error('Error fetching all channel data:', error);
      throw error;
    }
  },

  getChannelData: async (channelName: string, maxSamples = 1000) => {
    try {
      const response = await axios.get(`${API_BASE_URL}/data/${channelName}`, {
        params: { max_samples: maxSamples }
      });
      return response.data;
    } catch (error) {
      console.error(`Error fetching data for channel ${channelName}:`, error);
      throw error;
    }
  },

  // Method to determine maximum data rate the API can support
  testMaxDataRate: async () => {
    // This is a simplified approach - in a real implementation, 
    // you would measure response times for different request frequencies
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
      
      return Math.min(Math.max(1, safeMaxHz), 30); // Between 1Hz and 30Hz
    } catch (error) {
      console.error('Error testing max data rate:', error);
      return 5; // Default to 5Hz if test fails
    }
  }
};