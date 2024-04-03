import csv

def write_to_csv(file_path, data_list):
    # Open the CSV file in write mode
    with open(file_path, 'w', newline='') as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)

        # Transpose the data list to convert it into a list of lists (columns)
        data_columns = [[item] for item in data_list]

        # Write the data columns to the CSV file
        csv_writer.writerows(data_columns)
if __name__ == "__main__":
    
    data = []
    
    for i in range(286):
        data.append(1)
    
    for i in range(1001):
        data.append(2)
    
    for i in range(786):
        data.append(3)
    
    for i in range(238):
        data.append(4)
    
    for i in range(71):
        data.append(5)
    

    # Specify the file path where you want to save the CSV file
    file_path = "SATscores.csv"

    # Call the function to write the list to the CSV file
    write_to_csv(file_path, data)

    print(f"Data has been written to {file_path}")