import math
import numpy as np
from ahrs.filters import Madgwick
import matplotlib.pyplot as plt




def read_data():

  gps_data = []  # Array to hold GPS data in the format [(time, [lat, lon, alt])]
  imu_data = []  # Array to hold IMU data in the format [(time, [ax, ay, az, gx, gy, gz])]

  # GPS data parsing
  for i in range(18):  # Assuming 18 files for GPS
      num = f"{i:03}"  # Formatting to 3 digits
      gps_file = open(f"C:/Users/aaren/Desktop/OREGON/rollover1/ORE_gps_0_{num}")
      file_lines = gps_file.readlines()

      for line in file_lines:
          values = line.split(',')
          if values[1] != "nan":
              time = float(values[0])
              lat = float(values[1])
              lon = float(values[2])
              alt = float(values[3].strip('\n'))
              gps_data.append((time, [lat, lon, alt]))

  gps_break_time = gps_data[-1][0] if gps_data else 0  # Last GPS timestamp

  for i in range(45):
    num = f"{i:03}"  # Formatting to 3 digits
    gps_file = open(f"C:/Users/aaren/Desktop/OREGON/ORE_gps_0_{num}")
    file_lines = gps_file.readlines()

    for line in file_lines:
      values = line.split(',')
      if (values[1]!="nan"):
        time = float(values[0])+gps_break_time
        lat = float(values[1])
        lon = float(values[2])
        alt = float(values[3].strip('\n'))
        gps_data.append((time, [lat, lon, alt]))


  for i in range(18):  # Assuming 18 files for GPS
      num = f"{i:03}"  # Formatting to 3 digits
      imu_file = open(f"C:/Users/aaren/Desktop/OREGON/rollover1/ORE_imu_0_{num}")
      file_lines = imu_file.readlines()

      for line in file_lines:
          values = line.split(',')
          if values[1] != "nan":
              time = float(values[0])  # Adjust time based on last GPS timestamp
              ax, ay, az, gx, gy, gz = map(float, values[1:7])  # Assuming these indices hold IMU data
              imu_data.append((time, [ax, ay, az, gx, gy, gz]))

  imu_break_time = imu_data[-1][0] if imu_data else 0  # Last GPS timestamp


  # IMU data parsing (assuming similar file structure and naming convention)
  for i in range(45):  # Assuming 45 files for IMU
      num = f"{i:03}"  # Formatting to 3 digits

      imu_file = open(f"C:/Users/aaren/Desktop/OREGON/ORE_imu_0_{num}")
      file_lines = imu_file.readlines()

      for line in file_lines:
          values = line.split(',')
          if values[1] != "nan":
              time = float(values[0]) + imu_break_time  # Adjust time based on last GPS timestamp
              ax, ay, az, gx, gy, gz = map(float, values[1:7])  # Assuming these indices hold IMU data
              imu_data.append((time, [ax, ay, az, gx, gy, gz]))

  return gps_data, imu_data


def parse_data():
  gps, imu = read_data()


  imu_index = 0
  prior_time = 0

  gps_points = []
  imu_points = []
  delta_t = []

  for gps_time, (lat, lon, alt) in gps:
    # Find the IMU measurement closest in time to the current GPS measurement
    # This assumes imu and gps lists are sorted by time and monotonically increasing
    closest_time_diff = float('inf')
    closest_imu_data = imu[0]

    while imu_index < len(imu) - 1:
      current_time_diff = abs(imu[imu_index][0] - gps_time)
      next_time_diff = abs(imu[imu_index + 1][0] - gps_time)

      if current_time_diff < closest_time_diff:
        closest_time_diff = current_time_diff
        closest_imu_data = imu[imu_index]

      if next_time_diff < current_time_diff:
        imu_index += 1

      else:
        break
    
    ax, ay, az, gx, gy, gz = closest_imu_data[1]
     
    gps_points.append((lat, lon, alt))
    imu_points.append((ax, ay, az, gx, gy, gz))
    delta_t.append(gps_time - prior_time)

    prior_time = gps_time
  
  return gps_points, imu_points, delta_t



def gps_to_enu(seed_lat, seed_lon, seed_alt, query_lat, query_lon, query_alt):
    """
    Convert GPS coordinates to ENU coordinates relative to a seed location.

    Parameters:
    seed_lat, seed_lon, seed_alt: Seed (reference) location in GPS coordinates (degrees, meters).
    query_lat, query_lon, query_alt: Query location in GPS coordinates (degrees, meters).

    Returns:
    Tuple of ENU coordinates: (east, north, up) in meters.
    """
    # Constants
    a = 6378137  # Earth's radius in meters
    e = 8.1819190842622e-2  # Earth's eccentricity

    def gps_to_ecef(lat, lon, alt):
        """Convert GPS to ECEF coordinates."""
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        N = a / math.sqrt(1 - e**2 * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = ((1 - e**2) * N + alt) * math.sin(lat_rad)
        return x, y, z

    def ecef_to_enu(x, y, z, seed_x, seed_y, seed_z):
        """Convert ECEF coordinates to ENU coordinates."""
        dx = x - seed_x
        dy = y - seed_y
        dz = z - seed_z

        lat_rad = math.radians(seed_lat)
        lon_rad = math.radians(seed_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)

        e = -sin_lon * dx + cos_lon * dy
        n = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        u = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return e, n, u

    # Convert both seed and query locations from GPS to ECEF
    seed_x, seed_y, seed_z = gps_to_ecef(seed_lat, seed_lon, seed_alt)
    query_x, query_y, query_z = gps_to_ecef(query_lat, query_lon, query_alt)

    # Convert query location from ECEF to ENU relative to seed location
    e, n, u = ecef_to_enu(query_x, query_y, query_z, seed_x, seed_y, seed_z)

    return e, n, u

def compute_orientations(imu_data,dt,):
    
    madgwick = Madgwick(gain_imu=0.033, gain_marg=0.041)

    orientations = []
    Q = np.array([1., 0., 0., 0.])  # Initial quaternion
    for i, (ax, ay, az, gx, gy, gz) in enumerate(imu_data):
        madgwick.Dt = dt[i]
        Q = madgwick.updateIMU(Q, np.array([gx, gy, gz]), np.array([ax, ay, az]))
        orientations.append(Q)
    return orientations


def main():
    
  

  # Prepare data for processing
  gps_points, imu_points, delta_t = parse_data()  # parse_data logic to be adjusted as needed
  print(f'Input size - GPS: {len(gps_points)}, IMU: {len(imu_points)}, Delta_t: {len(delta_t)}')

  # Use the first GPS measurement as the seed for ENU conversion
  seed_lat, seed_lon, seed_alt = gps_points[0]
    
  orientations = compute_orientations(imu_points, delta_t)

  local = []
  for i, ((lat, lon, alt), orientation) in enumerate(zip(gps_points, orientations)):
    # Convert GPS data to ENU coordinates
    e, n, u = gps_to_enu(seed_lat, seed_lon, seed_alt, lat, lon, alt)
        
    # Append the combined data
    local.append((e, n, u, orientation[0], orientation[1], orientation[2], orientation[3]))
    
  return local

if __name__ == "__main__":
  local = main()
  print(len(local))
  print(len(local[0]))

  f = open("sam_lihn.txt", "w")
  for i in range(0, len(local)):
    s = ""
    for j in range(6):
      s+= str(local[i][j]) + ','
    s += str(local[i][-1])  + '\n'
    f.writelines([s])
  f.close()


