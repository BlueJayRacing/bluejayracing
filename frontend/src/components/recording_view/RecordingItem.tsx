// src/components/recording_view/RecordingItem.tsx
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { 
  Card, 
  CardContent, 
  Typography, 
  IconButton, 
  Button, 
  Menu, 
  MenuItem, 
  Dialog, 
  DialogTitle, 
  DialogContent, 
  DialogActions, 
  TextField, 
  Box, 
  Chip,
  Stack,
  Divider
} from '@mui/material';
import MoreVertIcon from '@mui/icons-material/MoreVert';
import PlayArrowIcon from '@mui/icons-material/PlayArrow';
import DeleteIcon from '@mui/icons-material/Delete';
import EditIcon from '@mui/icons-material/Edit';
import { useDataContext } from '../../hooks/useDataContext';
import { Recording } from '../shared/types';

interface RecordingItemProps {
  recording: Recording;
  onPlay?: (recording: Recording) => void;
}

const RecordingItem: React.FC<RecordingItemProps> = ({ recording, onPlay }) => {
  const navigate = useNavigate();
  const { deleteRecording, renameRecording } = useDataContext();
  const [menuAnchor, setMenuAnchor] = useState<null | HTMLElement>(null);
  const [renameDialogOpen, setRenameDialogOpen] = useState(false);
  const [deleteDialogOpen, setDeleteDialogOpen] = useState(false);
  const [newName, setNewName] = useState(recording.name);
  
  // Format date for display
  const formatDate = (timestamp: number) => {
    try {
      const date = new Date(timestamp);
      if (isNaN(date.getTime())) {
        return "Invalid date";
      }
      return date.toLocaleDateString() + ' ' + date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    } catch (e) {
      return "Date error";
    }
  };
  
  // Format file size
  const formatSize = (bytes: number) => {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  };
  
  // Format duration
  const formatDuration = (ms: number) => {
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
  };

  const handlePlayClick = () => {
    if (onPlay) {
      onPlay(recording);
    } else {
      navigate(`/playback/${recording.id}`);
    }
  };

  const handleDeleteRecording = () => {
    if (deleteRecording) {
      deleteRecording(recording.id);
    }
    setDeleteDialogOpen(false);
  };

  const handleRenameRecording = () => {
    if (renameRecording && newName.trim()) {
      renameRecording(recording.id, newName.trim());
      setRenameDialogOpen(false);
    }
  };
  
  return (
    <Card 
      variant="outlined" 
      sx={{ 
        transition: 'box-shadow 0.3s', 
        '&:hover': { 
          boxShadow: 3 
        },
        display: 'flex',
        flexDirection: 'column',
        height: '100%'
      }}
    >
      <CardContent sx={{ p: 2, pb: 1, flexGrow: 0 }}>
        {/* Header section with title and menu */}
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 1 }}>
          <Typography variant="h6" component="h3" sx={{ fontWeight: 'medium' }}>
            {recording.name}
          </Typography>
          <IconButton 
            onClick={(e: any) => setMenuAnchor(e.currentTarget)} 
            size="small"
            sx={{ mt: -0.5, mr: -0.5 }}
          >
            <MoreVertIcon />
          </IconButton>
        </Box>
        
        {/* Date information */}
        <Typography variant="body2" color="text.secondary" sx={{ mb: 2 }}>
          {formatDate(recording.startTime)}
          {recording.endTime ? ` - ${formatDate(recording.endTime)}` : ' (In progress)'}
        </Typography>
        
        <Divider sx={{ my: 1.5 }} />
        
        {/* Metrics display */}
        <Stack 
          direction="row" 
          spacing={1} 
          sx={{ 
            flexWrap: 'wrap', 
            gap: 1, 
            mb: 2,
            '& > *': { 
              flexGrow: 1,
              minWidth: 'calc(50% - 4px)', // Two chips per row on smaller cards
              '@media (min-width: 400px)': {
                minWidth: 'auto' // Let them flow naturally on wider cards
              }
            }
          }}
        >
          <Chip 
            label={`Duration: ${formatDuration(recording.stats.duration)}`} 
            variant="outlined" 
            size="small"
          />
          <Chip 
            label={`Channels: ${recording.stats.channelCount}`} 
            variant="outlined"
            size="small"
          />
          <Chip 
            label={`Samples: ${recording.stats.sampleCount.toLocaleString()}`} 
            variant="outlined"
            size="small"
          />
          <Chip 
            label={`Size: ${formatSize(recording.stats.dataSize)}`} 
            variant="outlined"
            size="small"
          />
        </Stack>
      </CardContent>
      
      {/* Action button section */}
      <Box sx={{ mt: 'auto', p: 2, pt: 0 }}>
        <Button 
          variant="contained" 
          color="primary"
          fullWidth
          startIcon={<PlayArrowIcon />}
          onClick={handlePlayClick}
        >
          View Recording
        </Button>
      </Box>
      
      {/* Menu */}
      <Menu
        anchorEl={menuAnchor}
        open={Boolean(menuAnchor)}
        onClose={() => setMenuAnchor(null)}
      >
        <MenuItem onClick={() => {
          setMenuAnchor(null);
          setRenameDialogOpen(true);
        }}>
          <EditIcon fontSize="small" sx={{ mr: 1 }} /> Rename
        </MenuItem>
        <MenuItem 
          onClick={() => {
            setMenuAnchor(null);
            setDeleteDialogOpen(true);
          }} 
          sx={{ color: 'error.main' }}
        >
          <DeleteIcon fontSize="small" sx={{ mr: 1 }} /> Delete
        </MenuItem>
      </Menu>
      
      {/* Rename Dialog */}
      <Dialog open={renameDialogOpen} onClose={() => setRenameDialogOpen(false)}>
        <DialogTitle>Rename Recording</DialogTitle>
        <DialogContent>
          <TextField
            autoFocus
            margin="dense"
            label="Name"
            fullWidth
            value={newName}
            onChange={(e) => setNewName(e.target.value)}
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setRenameDialogOpen(false)}>Cancel</Button>
          <Button 
            onClick={handleRenameRecording} 
            color="primary"
            variant="contained"
          >
            Save
          </Button>
        </DialogActions>
      </Dialog>
      
      {/* Delete Confirmation Dialog */}
      <Dialog open={deleteDialogOpen} onClose={() => setDeleteDialogOpen(false)}>
        <DialogTitle>Delete Recording</DialogTitle>
        <DialogContent>
          <Typography>Are you sure you want to delete "{recording.name}"?</Typography>
          <Typography variant="caption" color="text.secondary" sx={{ display: 'block', mt: 2 }}>
            This action cannot be undone.
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setDeleteDialogOpen(false)}>Cancel</Button>
          <Button 
            onClick={handleDeleteRecording} 
            color="error"
            variant="contained"
          >
            Delete
          </Button>
        </DialogActions>
      </Dialog>
    </Card>
  );
};

export default RecordingItem;