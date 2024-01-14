import React, { useState, useEffect } from 'react';
import PropTypes from 'prop-types';
import { Table, TableBody, TableCell, TableContainer, TableHead, TableRow, TableSortLabel, Checkbox, Paper, Typography, Box, TextField } from '@mui/material';



function descendingComparator(a, b, orderBy) {
  if (b[orderBy] < a[orderBy]) {
    return -1;
  }
  if (b[orderBy] > a[orderBy]) {
    return 1;
  }
  return 0;
}

function getComparator(order, orderBy) {
  return order === 'desc'
    ? (a, b) => descendingComparator(a, b, orderBy)
    : (a, b) => -descendingComparator(a, b, orderBy);
}

function stableSort(array, comparator) {
  const stabilizedThis = array.map((el, index) => [el, index]);
  stabilizedThis.sort((a, b) => {
    const order = comparator(a[0], b[0]);
    if (order !== 0) {
      return order;
    }
    return a[1] - b[1];
  });
  return stabilizedThis.map((el) => el[0]);
}

function EnhancedTableHead(props) {
  const { order, orderBy, onRequestSort } = props;
  const headers = [ 'FileName','Name','PartNum','id','COTS','COTSnum', 'Revision', 'Material', 'Condition','Owner','Vendor','QTYonCar','Model', 'Analysis', 'Drawing','Drawing Rev','PDF', 'DXF', 'Order Date','Weight','Assembly Weight', 'QTYcomplete-A01', 'QTYcomplete-A02', 'QTYcomplete-A03', 'QTYcomplete-A04' ];

  const createSortHandler = (property) => (event) => {
    onRequestSort(event, property);
  };

  return (
    <TableHead>
      <TableRow>
        {headers.map((header) => (
          <TableCell key={header}>
            <TableSortLabel
              active={orderBy === header}
              direction={orderBy === header ? order : 'asc'}
              onClick={createSortHandler(header)}
            >
              {header}
            </TableSortLabel>
          </TableCell>
        ))}
      </TableRow>
    </TableHead>
  );
}

EnhancedTableHead.propTypes = {
  onRequestSort: PropTypes.func.isRequired,
  order: PropTypes.oneOf(['asc', 'desc']).isRequired,
  orderBy: PropTypes.string.isRequired,
};

function StickyHeadTable() {
  const [order, setOrder] = useState('asc');
  const [orderBy, setOrderBy] = useState('PartNum');
  const [data, setData] = useState([]);

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

  function processApiResponse(apiResponse) {
    // Extract and format the required fields
    if (apiResponse.results && Array.isArray(apiResponse.results)) {
      return apiResponse.results.filter(function(row) {return !row.hideRow}).map(function(rowData) {
        return {
          FileName: rowData.FileName,
          Name: rowData.Name,
          PartNum: rowData.PartNum,
          id: rowData.id,
          COTS: rowData.COTS.value,
          COTSColor: rowData.COTS.color,
          COTSnum: rowData.COTSnum,
          Revision: rowData.Revision,
          Material: rowData.Material,
          Condition: rowData.Condition,
          Owner: rowData.Owner.value,
          OwnerColor: rowData.Owner.color,
          Vendor: rowData.Vendor,
          QTYonCar: rowData.QTYonCar,
          Model: rowData.Model.value,
          ModelColor: rowData.Model.color,
          Analysis: rowData.Analysis.value,
          AnalysisColor: rowData.Analysis.color,
          Drawing: rowData.Drawing.value,
          DrawingColor: rowData.Drawing.color,
          'Drawing Rev': rowData['Drawing Rev'],
          PDF: rowData.PDF.value,
          PDFColor: rowData.PDF.color,
          DXF: rowData.DXF.value,
          DXFColor: rowData.DXF.color,
          'Order Date': rowData['Order Date'],
          Weight: rowData.Weight,
          'Assembly Weight': rowData['Assembly Weight'],
          'QTYcomplete-A01': rowData['QTYcomplete-A01'],
          'QTYcomplete-A02': rowData['QTYcomplete-A02'],
          'QTYcomplete-A03': rowData['QTYcomplete-A03'],
          'QTYcomplete-A04': rowData['QTYcomplete-A04'],
        };
      });
    } else {
      console.log("No 'results' array found in the response");
      return [];
    }
  }
  function BasicTextFields() {
    return (
      <Box
        component="form"
        sx={{
          '& > :not(style)': { m: 1, width: '25ch' },
        }}
        noValidate
        autoComplete="off"
      >
        <TextField id="outlined-basic" label="Subsystem" variant="outlined" />
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