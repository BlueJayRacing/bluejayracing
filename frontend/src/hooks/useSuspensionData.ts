import { useEffect, useContext } from 'react';
import { SuspensionContext } from '../contexts/SuspensionContext';

const useSuspensionData = () => {
  const context = useContext(SuspensionContext);

  useEffect(() => {
    if (!context) return;

    const { setShockExtension } = context;

    const fetchSuspensionData = async () => {
      try {
        const response = await fetch('/api/suspension-data'); // Replace with your API endpoint
        const data = await response.json();
        setShockExtension(data.shockExtension);
      } catch (error) {
        console.error('Error fetching suspension data:', error);
      }
    };

    // const interval = setInterval(fetchSuspensionData, 200); // 5Hz polling (every 200ms)

    return () => {
    //   clearInterval(interval);
    };
  }, [context]);
};

export default useSuspensionData;
