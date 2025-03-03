// src/utils/indexedDbStorage.ts
import { Recording } from '../components/shared/types';
import { 
  isValidNumber, 
  isValidString, 
  isValidArray, 
  isValidObject 
} from './dataTypes';

interface StorageSchema {
  recordings: Recording;
  // Add other types of data you want to store
}

class IndexedDBStorage {
  private dbName = 'BlueJayRacingDB';
  private dbVersion = 1;
  private db: IDBDatabase | null = null;

  constructor() {
    this.init();
  }

  private init(): Promise<IDBDatabase> {
    return new Promise((resolve, reject) => {
      const request = indexedDB.open(this.dbName, this.dbVersion);

      request.onerror = (event) => {
        console.error('IndexedDB error:', event);
        reject(new Error('Failed to open IndexedDB'));
      };

      request.onsuccess = (event) => {
        this.db = (event.target as IDBOpenDBRequest).result;
        resolve(this.db);
      };

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result;
        
        // Create object stores
        if (!db.objectStoreNames.contains('recordings')) {
          db.createObjectStore('recordings', { keyPath: 'id' });
        }
        // Add more object stores as needed
      };
    });
  }

  private async ensureDB(): Promise<IDBDatabase> {
    if (!this.db) {
      await this.init();
    }
    return this.db!;
  }

  public async getItem<K extends keyof StorageSchema>(
    storeName: K, 
    key: string
  ): Promise<StorageSchema[K] | null> {
    try {
      const db = await this.ensureDB();
      return new Promise((resolve, reject) => {
        const transaction = db.transaction([storeName], 'readonly');
        const store = transaction.objectStore(storeName);
        const request = store.get(key);

        request.onsuccess = () => {
          resolve(request.result || null);
        };

        request.onerror = () => {
          console.error(`Error retrieving ${key} from ${storeName}`);
          reject(null);
        };
      });
    } catch (error) {
      console.error('IndexedDB retrieval error:', error);
      return null;
    }
  }

  public async setItem<K extends keyof StorageSchema>(
    storeName: K, 
    value: StorageSchema[K]
  ): Promise<boolean> {
    try {
      const db = await this.ensureDB();
      return new Promise((resolve) => {
        const transaction = db.transaction([storeName], 'readwrite');
        const store = transaction.objectStore(storeName);
        const request = store.put(value);

        request.onsuccess = () => {
          resolve(true);
        };

        request.onerror = (event) => {
          console.error('Error storing item:', event);
          resolve(false);
        };
      });
    } catch (error) {
      console.error('IndexedDB storage error:', error);
      return false;
    }
  }

  public async getAllItems<K extends keyof StorageSchema>(
    storeName: K
  ): Promise<StorageSchema[K][]> {
    try {
      const db = await this.ensureDB();
      return new Promise((resolve, reject) => {
        const transaction = db.transaction([storeName], 'readonly');
        const store = transaction.objectStore(storeName);
        const request = store.getAll();

        request.onsuccess = () => {
          resolve(request.result || []);
        };

        request.onerror = () => {
          console.error(`Error retrieving all items from ${storeName}`);
          reject([]);
        };
      });
    } catch (error) {
      console.error('IndexedDB retrieval error:', error);
      return [];
    }
  }

  public async removeItem<K extends keyof StorageSchema>(
    storeName: K, 
    key: string
  ): Promise<boolean> {
    try {
      const db = await this.ensureDB();
      return new Promise((resolve) => {
        const transaction = db.transaction([storeName], 'readwrite');
        const store = transaction.objectStore(storeName);
        const request = store.delete(key);

        request.onsuccess = () => {
          resolve(true);
        };

        request.onerror = (event) => {
          console.error('Error removing item:', event);
          resolve(false);
        };
      });
    } catch (error) {
      console.error('IndexedDB removal error:', error);
      return false;
    }
  }

  // Specific method for recordings
  public async saveRecordings(recordings: Recording[]): Promise<boolean> {
    try {
      const db = await this.ensureDB();
      const transaction = db.transaction(['recordings'], 'readwrite');
      const store = transaction.objectStore('recordings');

      // Validate recordings before saving
      const validRecordings = recordings.filter(recording => {
        return isValidString(recording.id) &&
               isValidNumber(recording.startTime) &&
               isValidObject(recording.channelData) &&
               isValidObject(recording.stats);
      });

      // Clear existing recordings first
      store.clear();

      // Save each validated recording
      validRecordings.forEach(recording => {
        store.put(recording);
      });

      return true;
    } catch (error) {
      console.error('Error saving recordings:', error);
      return false;
    }
  }

  public async getRecordings(): Promise<Recording[]> {
    try {
      const db = await this.ensureDB();
      return new Promise((resolve, reject) => {
        const transaction = db.transaction(['recordings'], 'readonly');
        const store = transaction.objectStore('recordings');
        const request = store.getAll();

        request.onsuccess = () => {
          const recordings = request.result || [];
          
          // Additional validation of recordings
          const validRecordings = recordings.filter(recording => 
            isValidString(recording.id) &&
            isValidNumber(recording.startTime) &&
            isValidObject(recording.channelData) &&
            isValidObject(recording.stats)
          );

          resolve(validRecordings);
        };

        request.onerror = () => {
          console.error('Error retrieving recordings');
          reject([]);
        };
      });
    } catch (error) {
      console.error('IndexedDB retrieval error:', error);
      return [];
    }
  }
}

// Export a singleton instance
export const IndexedStorage = new IndexedDBStorage();