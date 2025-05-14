// src/components/shared/ApiService.ts

import { API_CONFIG } from '../../config/deviceConfig';

// Response structure for API calls
export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
}

// Sample interface (can be modified based on actual API structure)
export interface Sample {
  channel_id: number;
  channel_index: number;
  data_value: number;
  timestamp: number;
}

// Data chunk interface
export interface DataChunk {
  aggregation_window: number;
  down_sampling: number;
  samples: Sample[];
  source_id: string;
}

// Channel mapping interface
export interface ChannelMapping {
  channel_id: number;
  channel_index: number;
  channel_name: string;
  source_id: string;
}

// API Service class
export class ApiService {
  private static baseUrl = API_CONFIG.baseUrl;

  /**
   * Helper method to handle API responses
   */
  private static async handleResponse<T>(response: Response): Promise<ApiResponse<T>> {
    if (!response.ok) {
      return {
        success: false,
        error: `API error: ${response.status} ${response.statusText}`
      };
    }

    try {
      const data = await response.json();
      return {
        success: true,
        data
      };
    } catch (error) {
      return {
        success: false,
        error: `Failed to parse response: ${error}`
      };
    }
  }

  /**
   * Get channel mapping from API
   */
  static async getChannelMapping(): Promise<ApiResponse<{ mappings: ChannelMapping[] }>> {
    try {
      const response = await fetch(`${this.baseUrl}${API_CONFIG.endpoints.mapping}`);
      return this.handleResponse<{ mappings: ChannelMapping[] }>(response);
    } catch (error) {
      console.error('Error fetching channel mapping:', error);
      return {
        success: false,
        error: `Failed to fetch channel mapping: ${error}`
      };
    }
  }

  /**
   * Get all channel data from all devices
   */
  static async getAllChannelData(): Promise<ApiResponse<{ data_chunks: DataChunk[] }>> {
    try {
      // Fixed URL construction - use the correct allData endpoint
      const response = await fetch(`${this.baseUrl}${API_CONFIG.endpoints.allData}`);
      return this.handleResponse<{ data_chunks: DataChunk[] }>(response);
    } catch (error) {
      console.error('Error fetching all channel data:', error);
      return {
        success: false,
        error: `Failed to fetch all channel data: ${error}`
      };
    }
  }

  /**
   * Get data for a specific device
   */
  static async getDeviceData(deviceId: string): Promise<ApiResponse<{ data_chunks: DataChunk[] }>> {
    try {
      // Fixed URL construction - make sure deviceId is appended correctly
      const response = await fetch(`${this.baseUrl}${API_CONFIG.endpoints.deviceData}${deviceId}`);
      return this.handleResponse<{ data_chunks: DataChunk[] }>(response);
    } catch (error) {
      console.error(`Error fetching data for device ${deviceId}:`, error);
      return {
        success: false,
        error: `Failed to fetch data for device ${deviceId}: ${error}`
      };
    }
  }

  /**
   * Get data for multiple channels
   */
  static async getChannelsData(
    channelIds: number[]
  ): Promise<ApiResponse<{ samples: Sample[] }>> {
    try {
      // Construct query parameters
      const channelParams = channelIds.map(id => `channel_id=${id}`).join('&');
      const url = `${this.baseUrl}${API_CONFIG.endpoints.deviceData}?${channelParams}`;
      
      const response = await fetch(url);
      return this.handleResponse<{ samples: Sample[] }>(response);
    } catch (error) {
      console.error('Error fetching channel data:', error);
      return {
        success: false,
        error: `Failed to fetch channel data: ${error}`
      };
    }
  }
}