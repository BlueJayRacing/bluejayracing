import React, { useState, useEffect } from 'react';
import PropTypes from 'prop-types';
import { Table, TableBody, TableCell, TableContainer, TableHead, TableRow, TableSortLabel, Checkbox, Paper, Typography, Box, TextField } from '@mui/material';
import { descendingComparator, getComparator, stableSort } from '../Scripts/Sorting';
import EnhancedTableHead from '../Scripts/EnhancedTableHeads';
import processApiResponse from '../Scripts/processApiResponse';
import BasicTextFields from '../Scripts/BasicTextFields';

function StickyHeadTable() {
  const [order, setOrder] = useState('asc');
  const [orderBy, setOrderBy] = useState('PartNum');
  const [data, setData] = useState([]);
  const [searchTerm, setSearchTerm] = useState('');

  const handleRequestSort = (event, property) => {
    const isAsc = orderBy === property && order === 'asc';
    setOrder(isAsc ? 'desc' : 'asc');
    setOrderBy(property);
  };

  useEffect(() => {
    async function fetchData() {
      const apiURL = "https://api.baserow.io/api/database/rows/table/166656/?user_field_names=true&size=200";
      const headers = {
        "Authorization": "Token 0PgjngmqYskoMqepoWjJcvQ6Id6h81LC",
        "Content-Type": "application/json"
      };
      const options = {
        "method": "get",
        "headers": headers
      };
      const response = await fetch(apiURL, options);
      const apiData = await response.json();
      const processedData = processApiResponse(apiData);
      setData(processedData);
    };

    fetchData();
  }, []);
  const filteredData = data.filter((item) =>
  Object.values(item).some((value) =>
    String(value).toLowerCase().includes(searchTerm.toLowerCase())
  )
);


function BasicTextFields() {
  const handleSearch = (event) => {
    setSearchTerm(event.target.value);
  };

  return (
    <Box
      component="form"
      sx={{
        '& > :not(style)': { m: 1, width: '25ch' },
      }}
      noValidate
      autoComplete="off"
    >
      <TextField id="outlined-basic" label="Subsystem" variant="outlined" onChange={handleSearch} />
        <TextField id="outlined-basic" label="Assembly" variant="outlined" />
        <TextField id="outlined-basic" label="Part Index" variant="outlined" />
        <TextField id="outlined-basic" label="COTS #" variant="outlined" />
        <TextField id="outlined-basic" label="Material" variant="outlined" />
      
      </Box>
    );
  }
  return (
   
   <Paper sx={{ width: '80%', margin: '0 auto', display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
    <BasicTextFields />
      <Typography variant="h6" component="div" sx={{ fontWeight: 'bold', padding: 1, textAlign: 'center' }}>
        Search Table
      </Typography>
      <TableContainer sx={{ maxHeight: 440 }}>
        <Table stickyHeader aria-label="sticky table">
          <EnhancedTableHead
            order={order}
            orderBy={orderBy}
            onRequestSort={handleRequestSort}
          />
          <TableBody>
  {stableSort(data, getComparator(order, orderBy))
    .map((row, index) => {
      return (
        <TableRow hover tabIndex={-1} key={index}>
          <TableCell>{row.FileName || ''}</TableCell>
          <TableCell>{row.Name || ''}</TableCell>
          <TableCell>{row.PartNum || ''}</TableCell>
          <TableCell>{row.id || ''}</TableCell>
          <TableCell style={{ color: row.COTS?.color }}>{row.COTS?.value || ''}</TableCell>
          <TableCell>{row.COTSnum || ''}</TableCell>
          <TableCell>{row.Revision || ''}</TableCell>
          <TableCell>{row.Material || ''}</TableCell>
          <TableCell>{row.Condition || ''}</TableCell>
          <TableCell style={{ color: row.Owner?.color }}>{row.Owner?.value || ''}</TableCell>
          <TableCell>{row.Vendor || ''}</TableCell>
          <TableCell>{row.QTYonCar || ''}</TableCell>
          <TableCell style={{ color: row.Model?.color }}>{row.Model?.value || ''}</TableCell>
          <TableCell style={{ color: row.Analysis?.color }}>{row.Analysis?.value || ''}</TableCell>
          <TableCell style={{ color: row.Drawing?.color }}>{row.Drawing?.value || ''}</TableCell>
          <TableCell>{row['Drawing Rev'] || ''}</TableCell>
          <TableCell style={{ color: row.PDF?.color }}>{row.PDF?.value || ''}</TableCell>
          <TableCell style={{ color: row.DXF?.color }}>{row.DXF?.value || ''}</TableCell>
          <TableCell>{row['Order Date'] || ''}</TableCell>
          <TableCell>{row.Weight || ''}</TableCell>
          <TableCell>{row['Assembly Weight'] || ''}</TableCell>
          <TableCell>{row['QTYcomplete-A01'] || ''}</TableCell>
          <TableCell>{row['QTYcomplete-A02'] || ''}</TableCell>
          <TableCell>{row['QTYcomplete-A03'] || ''}</TableCell>
          <TableCell>{row['QTYcomplete-A04'] || ''}</TableCell>
        </TableRow>
      );
    })}
</TableBody>
        </Table>
      </TableContainer>
    </Paper>
  );
}

export default StickyHeadTable;