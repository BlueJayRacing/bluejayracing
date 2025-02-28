// src/utils/dataTypes.ts
// Utility functions for validating data types and structures

/**
 * Check if a value is a valid number
 */
export const isValidNumber = (value: any): boolean => {
  return typeof value === 'number' && !isNaN(value) && isFinite(value);
};

/**
 * Check if a value is a valid string
 */
export const isValidString = (value: any): boolean => {
  return typeof value === 'string' && value.trim().length > 0;
};

/**
 * Check if a value is a valid array
 */
export const isValidArray = (value: any): boolean => {
  return Array.isArray(value) && value.length > 0;
};

/**
 * Check if a value is a valid object
 */
export const isValidObject = (value: any): boolean => {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
};

/**
 * Check if a timestamp is valid (a number that represents a date)
 */
export const isValidTimestamp = (timestamp: any): boolean => {
  if (!isValidNumber(timestamp)) return false;
  
  // Make sure it's within a reasonable range (between 2020 and 2050)
  const date = new Date(timestamp);
  const year = date.getFullYear();
  return year >= 2020 && year <= 2050;
};

/**
 * Safely access a property from an object with optional chaining
 */
export const safeGet = <T>(obj: any, path: string, defaultValue: T): T => {
  try {
    const parts = path.split('.');
    let current = obj;
    
    for (const part of parts) {
      if (current === null || current === undefined) {
        return defaultValue;
      }
      current = current[part];
    }
    
    return (current === null || current === undefined) ? defaultValue : current as T;
  } catch (error) {
    return defaultValue;
  }
};

/**
 * Safely parse JSON
 */
export const safeParseJSON = <T>(json: string, defaultValue: T): T => {
  try {
    return JSON.parse(json) as T;
  } catch (error) {
    return defaultValue;
  }
};

/**
 * Safely access localStorage
 */
export const safeLocalStorage = {
  getItem: (key: string): string | null => {
    try {
      return localStorage.getItem(key);
    } catch (error) {
      console.error('Error accessing localStorage:', error);
      return null;
    }
  },
  
  setItem: (key: string, value: string): boolean => {
    try {
      localStorage.setItem(key, value);
      return true;
    } catch (error) {
      console.error('Error writing to localStorage:', error);
      return false;
    }
  },
  
  removeItem: (key: string): boolean => {
    try {
      localStorage.removeItem(key);
      return true;
    } catch (error) {
      console.error('Error removing from localStorage:', error);
      return false;
    }
  },
  
  getObject: <T>(key: string, defaultValue: T): T => {
    const json = safeLocalStorage.getItem(key);
    if (!json) return defaultValue;
    return safeParseJSON(json, defaultValue);
  },
  
  setObject: <T>(key: string, value: T): boolean => {
    try {
      const json = JSON.stringify(value);
      return safeLocalStorage.setItem(key, json);
    } catch (error) {
      console.error('Error serializing object for localStorage:', error);
      return false;
    }
  }
};