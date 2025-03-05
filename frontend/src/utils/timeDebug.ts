// src/utils/timeDebug.ts

/**
 * Utility function to safely debug timestamps
 * @param timestamp Any timestamp value to debug
 * @returns Debug string with information about the timestamp
 */
export const debugTimestamp = (timestamp: any): string => {
  try {
    const result = [];
    result.push(`Raw value: ${timestamp}`);
    result.push(`Type: ${typeof timestamp}`);
    
    // Try to parse as number
    const numValue = Number(timestamp);
    result.push(`As number: ${numValue}`);
    result.push(`Is NaN: ${isNaN(numValue)}`);
    
    // If it's a valid number, try as date
    if (!isNaN(numValue)) {
      try {
        // Try as milliseconds
        let dateMs = new Date(numValue);
        result.push(`Valid as ms: ${!isNaN(dateMs.getTime())}`);
        if (!isNaN(dateMs.getTime())) {
          result.push(`As date (ms): ${dateMs.toLocaleString()}`);
        }
        
        // Try as nanoseconds converted to ms
        let dateNano = new Date(Math.floor(numValue / 1000000));
        result.push(`Valid as ns: ${!isNaN(dateNano.getTime())}`);
        if (!isNaN(dateNano.getTime())) {
          result.push(`As date (ns): ${dateNano.toLocaleString()}`);
        }
        
        // Guess timestamp type
        if (numValue < 10000000000) {
          result.push(`Likely format: Unix timestamp (seconds)`);
        } else if (numValue < 10000000000000) {
          result.push(`Likely format: Unix timestamp (milliseconds)`);
        } else {
          result.push(`Likely format: Unix timestamp (microseconds/nanoseconds)`);
        }
      } catch (e) {
        result.push(`Date conversion error: ${e.message}`);
      }
    }
    
    return result.join(' | ');
  } catch (e) {
    return `Timestamp debug error: ${e.message}`;
  }
};

/**
 * Utility to safely format a date without throwing errors
 * @param timestamp Timestamp to format
 * @param fallback Fallback value if formatting fails
 * @returns Formatted date string or fallback
 */
export const safeFormatDate = (timestamp: any, fallback: string = "Invalid date"): string => {
  try {
    // First try as a regular millisecond timestamp
    const numValue = Number(timestamp);
    if (isNaN(numValue)) return fallback;
    
    // Try as milliseconds
    const date = new Date(numValue);
    if (!isNaN(date.getTime())) {
      return date.toLocaleString();
    }
    
    // If that fails, try as nanoseconds
    if (numValue > 1000000000000000) { // > year 33658 in ms = likely nanoseconds
      const dateNano = new Date(Math.floor(numValue / 1000000));
      if (!isNaN(dateNano.getTime())) {
        return dateNano.toLocaleString() + " (ns)";
      }
    }
    
    return fallback;
  } catch (e) {
    return fallback;
  }
};

/**
 * Force convert any timestamp to a usable number
 * @param timestamp Any timestamp value
 * @returns A numeric millisecond timestamp or null if invalid
 */
export const normalizeTimestamp = (timestamp: any): number | null => {
  try {
    // If it's already a number
    if (typeof timestamp === 'number') {
      // Convert nanoseconds to milliseconds if needed
      if (timestamp > 1000000000000000) { // > year 33658 in ms = likely nanoseconds
        return Math.floor(timestamp / 1000000);
      }
      // Convert seconds to milliseconds if needed
      else if (timestamp < 10000000000) { // Less than year 2286 in seconds
        return timestamp * 1000;
      }
      return timestamp; // Already in milliseconds
    }
    
    // If it's a string, try to parse it
    if (typeof timestamp === 'string') {
      // Try as a number first
      const numValue = Number(timestamp);
      if (!isNaN(numValue)) {
        // Convert nanoseconds to milliseconds if needed
        if (numValue > 1000000000000000) {
          return Math.floor(numValue / 1000000);
        }
        // Convert seconds to milliseconds if needed
        else if (numValue < 10000000000) {
          return numValue * 1000;
        }
        return numValue; // Already in milliseconds
      }
      
      // Try as a date string
      const dateValue = new Date(timestamp).getTime();
      if (!isNaN(dateValue)) {
        return dateValue;
      }
    }
    
    // If it's a Date object
    if (timestamp instanceof Date) {
      return timestamp.getTime();
    }
    
    // Give up
    return null;
  } catch (e) {
    console.error("Error normalizing timestamp:", e);
    return null;
  }
};