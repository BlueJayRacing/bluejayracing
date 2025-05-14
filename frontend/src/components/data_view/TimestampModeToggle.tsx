// src/components/data_view/TimestampModeToggle.tsx

import React from 'react';
import { 
  Box, 
  FormControlLabel, 
  Switch, 
  Typography, 
  Tooltip 
} from '@mui/material';
import AccessTimeIcon from '@mui/icons-material/AccessTime';
import UpdateIcon from '@mui/icons-material/Update';
import { useTimestamp } from '../../contexts/TimestampContext';

const TimestampModeToggle: React.FC = () => {
  const { mode, setMode } = useTimestamp();

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setMode(event.target.checked ? 'relative' : 'absolute');
  };

  return (
    <Box className="flex items-center" sx={{ ml: 2 }}>
      <Tooltip title="Absolute timestamps show real time values. Relative timestamps adjust so the most recent data appears as 'now'">
        <Box className="flex items-center">
          <AccessTimeIcon fontSize="small" color={mode === 'absolute' ? 'primary' : 'disabled'} sx={{ mr: 0.5 }} />
          
          <FormControlLabel
            control={
              <Switch
                checked={mode === 'relative'}
                onChange={handleChange}
                size="small"
              />
            }
            label=""
          />
          
          <UpdateIcon fontSize="small" color={mode === 'relative' ? 'primary' : 'disabled'} sx={{ ml: 0.5 }} />
          
          <Typography variant="body2" color="text.secondary" sx={{ ml: 1 }}>
            {mode === 'relative' ? 'Relative' : 'Absolute'} Time
          </Typography>
        </Box>
      </Tooltip>
    </Box>
  );
};

export default TimestampModeToggle;