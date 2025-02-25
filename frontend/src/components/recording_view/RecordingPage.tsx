// RecordingPage.tsx
import React from 'react';
import { useDataContext } from '../../hooks/useDataContext';
import RecordingItem from './RecordingItem';
import RecordingTimeline from './RecordingTimeline';

const RecordingPage: React.FC = () => {
  const { recordings } = useDataContext();

  return (
    <div className="container mx-auto p-4">
      <h1 className="text-2xl font-bold mb-6">Recordings</h1>
      
      {/* Timeline overview */}
      <div className="mb-8">
        <h2 className="text-xl font-semibold mb-4">Timeline</h2>
        <RecordingTimeline recordings={recordings} />
      </div>
      
      {/* Recordings list */}
      <div>
        <h2 className="text-xl font-semibold mb-4">All Recordings</h2>
        {recordings.length === 0 ? (
          <p className="text-gray-500">No recordings yet. Go to the Data page to create recordings.</p>
        ) : (
          <div className="space-y-4">
            {recordings.map(recording => (
              <RecordingItem 
                key={recording.id} 
                recording={recording} 
              />
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default RecordingPage;