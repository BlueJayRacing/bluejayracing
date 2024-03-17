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

  export default BasicTextFields;