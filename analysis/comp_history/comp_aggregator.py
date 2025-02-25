import os
import csv
import matplotlib.pyplot as plt

def load_competition_data(file_path):
    competition_data = {}
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file)
        headers = next(csv_reader)
        for row in csv_reader:
            if len(row) >= 14:
                rank = int(row[0] if row[0] else 999)
                team = row[2]

                data = {
                    'Rank': rank,
                    'Car #': row[1],
                    'Team': row[3],
                    'Overall': float(row[4]) if row[4] else -1.0,
                    'Overall Dynamic': float(row[5]) if row[5] else -1.0,
                    'Overall Static': float(row[6] if row[6] else -1.0),
                    'Cost Event': float(row[7]) if row[7] else -1.0,
                    'Design': float(row[8]) if row[8] else -1.0,
                    'Business Presentation': float(row[9]) if row[9] else -1.0,
                    'Acceleration': float(row[10]) if row[10] else -1.0,
                    'Maneuverability': float(row[11]) if row[11] else -1.0,
                    'Hill Climb/Sled Pull': float(row[12]) if row[12] else -1.0,
                    'Suspension': float(row[13]) if row[13] else -1.0,
                    'Endurance': float(row[14]) if row[14] else -1.0,
                }
                if team in competition_data:
                    print(f"Duplicate team: {team}")
                    continue
                competition_data[team] = data
            else:
                print(f"Invalid row: {row}")
    return competition_data

def calculate_percentile(rank, num_cars):
    return (num_cars - rank) / num_cars * 100

def analyze_team_performance(team_name, competitions):
    team_stats = []
    for competition, data in competitions.items():
        if team_name in data:
            team_data = data[team_name]
            num_cars = len(data)

            # Sort the data based on each category
            overall_rank = sorted(data.values(), key=lambda x: x['Overall'], reverse=True).index(team_data) + 1
            overall_percentile = calculate_percentile(overall_rank, num_cars)
              
            dynamic_rank = sorted(data.values(), key=lambda x: x['Overall Dynamic'], reverse=True).index(team_data) + 1
            dynamic_percentile = calculate_percentile(dynamic_rank, num_cars)

            static_rank = sorted(data.values(), key=lambda x: x['Overall Static'], reverse=True).index(team_data) + 1
            static_percentile = calculate_percentile(static_rank, num_cars)

            cost_rank = sorted(data.values(), key=lambda x: x['Cost Event'], reverse=True).index(team_data) + 1
            cost_percentile = calculate_percentile(cost_rank, num_cars)

            design_rank = sorted(data.values(), key=lambda x: x['Design'], reverse=True).index(team_data) + 1
            design_percentile = calculate_percentile(design_rank, num_cars)

            business_rank = sorted(data.values(), key=lambda x: x['Business Presentation'], reverse=True).index(team_data) + 1
            business_percentile = calculate_percentile(business_rank, num_cars)

            acceleration_rank = sorted(data.values(), key=lambda x: x['Acceleration'], reverse=True).index(team_data) + 1
            acceleration_percentile = calculate_percentile(acceleration_rank, num_cars)

            maneuverability_rank = sorted(data.values(), key=lambda x: x['Maneuverability'], reverse=True).index(team_data) + 1
            maneuverability_percentile = calculate_percentile(maneuverability_rank, num_cars)

            hill_climb_rank = sorted(data.values(), key=lambda x: x['Hill Climb/Sled Pull'], reverse=True).index(team_data) + 1
            hill_climb_percentile = calculate_percentile(hill_climb_rank, num_cars)

            suspension_rank = sorted(data.values(), key=lambda x: x['Suspension'], reverse=True).index(team_data) + 1
            suspension_percentile = calculate_percentile(suspension_rank, num_cars)

            endurance_rank = sorted(data.values(), key=lambda x: x['Endurance'], reverse=True).index(team_data) + 1
            endurance_percentile = calculate_percentile(endurance_rank, num_cars)

            stats = {
                'Competition': competition,
                'Overall Rank': overall_rank,
                'Overall Percentile': overall_percentile,
                'Dynamic Rank': dynamic_rank,
                'Dynamic Percentile': dynamic_percentile,
                'Static Rank': static_rank,
                'Static Percentile': static_percentile,
                'Cost Rank': cost_rank,
                'Cost Percentile': cost_percentile,
                'Design Rank': design_rank,
                'Design Percentile': design_percentile,
                'Business Rank': business_rank,
                'Business Percentile': business_percentile,
                'Acceleration Rank': acceleration_rank,
                'Acceleration Percentile': acceleration_percentile,
                'Maneuverability Rank': maneuverability_rank,
                'Maneuverability Percentile': maneuverability_percentile,
                'Hill Climb/Sled Pull Rank': hill_climb_rank,
                'Hill Climb/Sled Pull Percentile': hill_climb_percentile,
                'Suspension Rank': suspension_rank,
                'Suspension Percentile': suspension_percentile,
                'Endurance Rank': endurance_rank,
                'Endurance Percentile': endurance_percentile,
                'Num Cars': num_cars,
            }
            team_stats.append(stats)
        else:
            # print(f"Team {team_name} not found in {competition}")
            stats = {
                'Competition':  competition,
            }
            team_stats.append(stats)
    return team_stats

def save_team_stats_to_csv(team_stats, output_file):
    headers = list(team_stats[0].keys())
    with open(output_file, 'w', newline='') as file:
        csv_writer = csv.DictWriter(file, fieldnames=headers)
        csv_writer.writeheader()
        csv_writer.writerows(team_stats)

def plot_team_performance(team_stats):
    competitions = [float(stats['Competition']) for stats in team_stats if 'Overall Percentile' in stats]
    competitions.reverse()
    overall_percentiles = [stats['Overall Percentile'] for stats in team_stats if 'Overall Percentile' in stats]
    overall_percentiles.reverse()

    plt.figure(figsize=(10, 6))
    plt.plot(competitions, overall_percentiles, marker='o')
    plt.xlabel('Competition')
    plt.ylabel('Overall Percentile')
    plt.title('Team Performance Over Competitions')
    plt.xticks(rotation=45)
    plt.grid(True)

    competitions = [float(stats['Competition']) for stats in team_stats if 'Overall Rank' in stats]
    competitions.reverse()
    overall_rank = [stats['Overall Rank'] for stats in team_stats if 'Overall Rank' in stats]
    overall_rank.reverse()

    plt.figure(figsize=(10, 6))
    plt.plot(competitions, overall_rank, marker='o')
    plt.xlabel('Competition')
    plt.ylabel('Overall Rank')
    plt.title('Team Performance Over Competitions')
    plt.xticks(rotation=45)
    plt.grid(True)
    plt.show()

# Get the list of competition files from the current directory
competition_files = [file for file in os.listdir() if file.startswith("Competition History")]
competition_files.reverse()
# competition_files = ["Competition History - 2022.3 - Arizona.csv"]


# Extract the date identifiers from the file names
date_identifiers = [file.split(" - ")[1] for file in competition_files]

# Load competition data
competitions = {}
for file, date in zip(competition_files, date_identifiers):
    competition_data = load_competition_data(file)
    competitions[date] = competition_data

# Analyze team performance
team_name = "Johns Hopkins Univ"
team_stats = analyze_team_performance(team_name, competitions)

# Save team stats to CSV
output_file = f"{team_name}_performance.csv"
save_team_stats_to_csv(team_stats, output_file)

# Plot team performance
plot_team_performance(team_stats)