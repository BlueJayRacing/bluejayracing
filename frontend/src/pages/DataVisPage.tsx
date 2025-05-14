// src/pages/DataVisPage.tsx
import React, { useState, useEffect } from 'react';
import { useDataContext } from '../hooks/useDataContext';
import DeviceSelector from '../components/data_view/DeviceSelector';
import ChannelSelector from '../components/data_view/ChannelSelector';
import MultiViewGraph from '../components/data_view/MultiViewGraph';
import RecordingControls from '../components/data_view/RecordingControls';
import SimpleChart from '../components/data_view/SimpleChartComponent';
import { Link } from 'react-router-dom';

// Interface for a graph configuration
interface GraphConfig {
  id: string;
  deviceId: string;
  selectedChannels: string[];
  expanded: boolean;
}

const DataVisPage: React.FC = () => {
  const { 
    channels, 
    isLoading,
    devices
  } = useDataContext();
  
  // UI state
  const [useSimpleCharts, setUseSimpleCharts] = useState(false);
  
  // Graph configuration state
  const [graphs, setGraphs] = useState<GraphConfig[]>([]);
  
  // Initialize with one graph when component mounts
  useEffect(() => {
    if (graphs.length === 0 && devices.length > 0) {
      // Find first available device
      const defaultDevice = devices.find(d => d.available) || devices[0];
      
      setGraphs([{
        id: `graph-${Date.now()}`,
        deviceId: defaultDevice.id,
        selectedChannels: [],
        expanded: true
      }]);
    }
  }, [devices, graphs.length]);
  
  // Update device for a specific graph
  const handleDeviceChange = (graphId: string, deviceId: string) => {
    setGraphs(prev => prev.map(graph => 
      graph.id === graphId 
        ? { ...graph, deviceId, selectedChannels: [] } 
        : graph
    ));
  };
  
  // Update channels for a specific graph
  const handleChannelChange = (graphId: string, channels: string[]) => {
    setGraphs(prev => prev.map(graph => 
      graph.id === graphId 
        ? { ...graph, selectedChannels: channels } 
        : graph
    ));
  };
  
  // Add a new graph
  const addGraph = () => {
    // Find first available device
    const defaultDevice = devices.find(d => d.available) || devices[0];
    
    setGraphs(prev => [
      ...prev, 
      {
        id: `graph-${Date.now()}`,
        deviceId: defaultDevice?.id || '',
        selectedChannels: [],
        expanded: true
      }
    ]);
  };
  
  // Remove a graph
  const removeGraph = (graphId: string) => {
    setGraphs(prev => prev.filter(graph => graph.id !== graphId));
  };
  
  // Toggle graph expansion/collapse
  const toggleGraphExpansion = (graphId: string) => {
    setGraphs(prev => prev.map(graph => 
      graph.id === graphId 
        ? { ...graph, expanded: !graph.expanded } 
        : graph
    ));
  };
  
  // Get device name for display
  const getDeviceName = (deviceId: string) => {
    const device = devices.find(d => d.id === deviceId);
    return device ? device.name : deviceId;
  };
  
  // Return loading indicator if data is still loading
  if (isLoading) {
    return (
      <div className="flex justify-center items-center h-screen">
        <div className="animate-spin rounded-full h-12 w-12 border-t-2 border-b-2 border-blue-500"></div>
      </div>
    );
  }
  
  return (
    <div className="min-h-screen bg-gray-50">
      {/* Header */}
      <header className="bg-blue-600 text-white px-4 py-2 flex justify-between items-center">
        <h1 className="text-xl font-bold">BlueJay Racing</h1>
        <nav className="flex space-x-4">
          <Link to="/" className="hover:underline">HOME</Link>
          <Link to="/data" className="hover:underline font-bold">DATA</Link>
          <Link to="/recordings" className="hover:underline">RECORDINGS</Link>
          <Link to="/graphing" className="hover:underline">GRAPHING</Link>
        </nav>
      </header>
      
      {/* Top Action Bar */}
      <div className="flex justify-between items-center p-4 bg-white border-b border-gray-200">
        <div className="flex items-center space-x-4">
          <button 
            onClick={addGraph}
            className="flex items-center px-3 py-2 bg-gray-800 text-white rounded hover:bg-gray-700 transition-colors"
          >
            <svg className="w-5 h-5 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6v6m0 0v6m0-6h6m-6 0H6" />
            </svg>
            ADD GRAPH
          </button>
          
          <div className="flex items-center">
            <span className="text-sm font-medium mr-2">Use Simple Charts</span>
            <label className="relative inline-flex items-center cursor-pointer">
              <input 
                type="checkbox" 
                className="sr-only peer"
                checked={useSimpleCharts}
                onChange={() => setUseSimpleCharts(!useSimpleCharts)}
              />
              <div className="w-11 h-6 bg-gray-200 rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-0.5 after:left-0.5 after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all peer-checked:bg-blue-600"></div>
            </label>
          </div>
        </div>
        
        <RecordingControls />
      </div>
      
      {/* Main Content */}
      <main className="p-4">
        {graphs.length === 0 ? (
          <div className="text-center py-12">
            <div className="text-gray-500 mb-4">No graphs added yet</div>
            <button 
              onClick={addGraph}
              className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
            >
              Add Your First Graph
            </button>
          </div>
        ) : (
          <div className="space-y-6">
            {graphs.map((graph, index) => (
              <div 
                key={graph.id} 
                className="border border-gray-200 rounded-md bg-white shadow-sm overflow-hidden"
              >
                {/* Graph Header */}
                <div 
                  className="bg-gray-50 border-b border-gray-200 px-4 py-3 flex justify-between items-center cursor-pointer"
                  onClick={() => toggleGraphExpansion(graph.id)}
                >
                  <div className="flex items-center">
                    <svg 
                      className={`w-5 h-5 mr-2 text-gray-500 transition-transform ${graph.expanded ? 'transform rotate-90' : ''}`} 
                      fill="none" 
                      viewBox="0 0 24 24" 
                      stroke="currentColor"
                    >
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
                    </svg>
                    <h2 className="text-lg font-medium">Graph {index + 1}</h2>
                    
                    {/* Device indicator */}
                    {graph.deviceId && (
                      <div className="ml-3 flex items-center">
                        <span className="text-sm text-gray-600 mr-1">Device:</span>
                        <span 
                          className={`px-2 py-0.5 rounded-full text-xs font-medium ${
                            devices.find(d => d.id === graph.deviceId)?.available
                              ? 'bg-green-100 text-green-800'
                              : 'bg-red-100 text-red-800'
                          }`}
                        >
                          {getDeviceName(graph.deviceId)}
                        </span>
                      </div>
                    )}
                    
                    {/* Channel count */}
                    <div className="ml-3">
                      <span className="text-sm text-gray-500">
                        {graph.selectedChannels.length} channel{graph.selectedChannels.length !== 1 ? 's' : ''} selected
                      </span>
                    </div>
                  </div>
                  
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      removeGraph(graph.id);
                    }}
                    className="p-1 text-gray-400 hover:text-red-500 rounded-full hover:bg-gray-100"
                    title="Remove Graph"
                  >
                    <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                    </svg>
                  </button>
                </div>
                
                {/* Graph Content */}
                {graph.expanded && (
                  <div className="flex flex-col md:flex-row">
                    {/* Left panel with selectors */}
                    <div className="w-full md:w-64 p-4 border-r border-gray-200">
                      <DeviceSelector 
                        selectedDeviceId={graph.deviceId}
                        onDeviceChange={(deviceId) => handleDeviceChange(graph.id, deviceId)} 
                      />
                      
                      <div className="my-4 border-t border-gray-200"></div>
                      
                      <ChannelSelector 
                        selectedDeviceId={graph.deviceId}
                        selectedChannels={graph.selectedChannels}
                        onSelectionChange={(channels) => handleChannelChange(graph.id, channels)} 
                      />
                    </div>
                    
                    {/* Right panel with graph */}
                    <div className="flex-1 p-4">
                      {graph.selectedChannels.length === 0 ? (
                        <div className="h-80 flex items-center justify-center bg-gray-50 rounded-md border border-gray-200">
                          <span className="text-gray-500">Select channels to display data</span>
                        </div>
                      ) : useSimpleCharts ? (
                        <div className="space-y-4">
                          {graph.selectedChannels.map((channelName) => (
                            <SimpleChart
                              key={channelName}
                              channelName={channelName}
                              height={80}
                              showValue={true}
                              timeWindow={30000}
                            />
                          ))}
                        </div>
                      ) : (
                        <div className="h-80 border border-gray-200 rounded-md bg-gray-50">
                          <MultiViewGraph
                            channelNames={graph.selectedChannels}
                            duration={30000}
                            height={320}
                            showLegend={true}
                            realtime={true}
                          />
                        </div>
                      )}
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </main>
    </div>
  );
};

export default DataVisPage;