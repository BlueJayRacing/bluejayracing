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


async function fetchData(setData) {
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

  export default fetchData; 