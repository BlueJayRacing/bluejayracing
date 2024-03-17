import React, { useState, useEffect } from 'react';
import PropTypes from 'prop-types';




function processApiResponse(apiResponse) {
    // Extract and format the required fields
    if (apiResponse.results && Array.isArray(apiResponse.results)) {
      return apiResponse.results.filter(function(row) {return !row.hideRow}).map(function(rowData) {
        return {
          FileName: rowData.FileName,
          Name: rowData.Name,
          PartNum: rowData.PartNum,
          id: rowData.id,
          COTS: rowData.COTS,
          COTSColor: rowData.COTS.color,
          COTSnum: rowData.COTSnum,
          Revision: rowData.Revision,
          Material: rowData.Material,
          Condition: rowData.Condition,
          Owner: rowData.Owner,
          OwnerColor: rowData.Owner.color,
          Vendor: rowData.Vendor,
          QTYonCar: rowData.QTYonCar,
          Model: rowData.Model,
          ModelColor: rowData.Model.color,
          Analysis: rowData.Analysis,
          AnalysisColor: rowData.Analysis.color,
          Drawing: rowData.Drawing,
          DrawingColor: rowData.Drawing.color,
          'Drawing Rev': rowData['Drawing Rev'],
          PDF: rowData.PDF,
          PDFColor: rowData.PDF.color,
          DXF: rowData.DXF,
          DXFColor: rowData.DXF.color,
          'Order Date': rowData['Order Date'],
          Weight: rowData.Weight,
          'Assembly Weight': rowData.AssyWeight,
          'QTYcomplete-A01': rowData['QTYcomplete-A01'],
          'QTYcomplete-A02': rowData['QTYcomplete-A02'],
          'QTYcomplete-A03': rowData['QTYcomplete-A03'],
          'QTYcomplete-A04': rowData['QTYcomplete-A04'],
        };
        console.log(rowData.COTSnum);
      });
    } else {
      console.log("No 'results' array found in the response");
      return [];
    }
  }

  export default processApiResponse;