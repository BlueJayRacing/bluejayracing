import React from 'react';
import TableHead from '@mui/material/TableHead';
import TableRow from '@mui/material/TableRow';
import TableCell from '@mui/material/TableCell';
import TableSortLabel from '@mui/material/TableSortLabel';
import PropTypes from 'prop-types';

function EnhancedTableHeads(props) {
  const { order, orderBy, onRequestSort } = props;
  const headers = [ 'FileName','Name','PartNum','id','COTS','COTS#', 'Revision', 'Material', 'Condition','Owner','Vendor','QTYonCar','Model', 'Analysis', 'Drawing','Drawing Rev','PDF', 'DXF', 'Order Date','Weight','Assembly Weight', 'QTYcomplete-A01', 'QTYcomplete-A02', 'QTYcomplete-A03', 'QTYcomplete-A04' ];

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

EnhancedTableHeads.propTypes = {
  onRequestSort: PropTypes.func.isRequired,
  order: PropTypes.oneOf(['asc', 'desc']).isRequired,
  orderBy: PropTypes.string.isRequired,
};

export default EnhancedTableHeads;