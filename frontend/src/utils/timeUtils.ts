// src/utils/timeUtils.ts

/**
 * Normalizes a timestamp to milliseconds
 * This handles various timestamp formats including nanoseconds and seconds
 */
export function normalizeTimestamp(timestamp: number | string): number {
  // Convert to number if it's a string
  const numericTimestamp = typeof timestamp === 'string' ? Number(timestamp) : timestamp;
  
  // Invalid timestamp
  if (isNaN(numericTimestamp)) {
    return 0;
  }
  
  // Already in milliseconds range (reasonable date between 1970 and 2100)
  if (numericTimestamp > 10000000000 && numericTimestamp < 4102444800000) {
    return numericTimestamp;
  }
  
  // Convert from nanoseconds
  if (numericTimestamp > 1000000000000000) { // > year 33658 in milliseconds
    return Math.floor(numericTimestamp / 1000000);
  }
  
  // Convert from seconds
  if (numericTimestamp < 10000000000) { // < year 2286 in milliseconds
    return numericTimestamp * 1000;
  }
  
  // Default case, return as is
  return numericTimestamp;
}

/**
 * Formats a timestamp as a readable date string
 */
export function formatTimestamp(timestamp: number, format: 'short' | 'medium' | 'long' = 'medium'): string {
  const normalizedTime = normalizeTimestamp(timestamp);
  const date = new Date(normalizedTime);
  
  // Invalid date
  if (isNaN(date.getTime())) {
    return 'Invalid date';
  }
  
  // Format based on requested length
  switch (format) {
    case 'short':
      return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    case 'long':
      return date.toLocaleString([], {
        year: 'numeric',
        month: 'long',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
      });
    case 'medium':
    default:
      return date.toLocaleString([], {
        month: 'short',
        day: 'numeric',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
      });
  }
}

/**
 * Formats a duration in milliseconds as a readable string
 */
export function formatDuration(ms: number): string {
  const seconds = Math.floor(ms / 1000);
  const minutes = Math.floor(seconds / 60);
  const hours = Math.floor(minutes / 60);
  
  if (hours > 0) {
    return `${hours}h ${minutes % 60}m ${seconds % 60}s`;
  }
  
  if (minutes > 0) {
    return `${minutes}m ${seconds % 60}s`;
  }
  
  return `${seconds}s`;
}

/**
 * Safely format a date - returns a helpful message if date is invalid
 */
export function safeFormatDate(timestamp: number): string {
  try {
    const date = new Date(timestamp);
    if (isNaN(date.getTime())) {
      return `Invalid date (${timestamp})`;
    }
    return date.toLocaleString();
  } catch (e) {
    return `Error formatting timestamp: ${timestamp}`;
  }
}

/**
 * Converts a byte count to a human-readable size string
 */
export function formatFileSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(1)} GB`;
}

export default {
  normalizeTimestamp,
  formatTimestamp,
  formatDuration,
  safeFormatDate,
  formatFileSize
};