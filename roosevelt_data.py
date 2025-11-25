"""
an adhoc script to visualize the data
"""

import pandas as pd
import folium
import matplotlib.pyplot as plt
import numpy as np
import os

### Globals
save_folder = ""

# Function to create the save folder for the plots
def create_save_folder():
    """
    Create the save folder where the plots will later be saved
    """

    # Check arguments
    assert isinstance(save_folder, str), "String path to folder was expected!"
    
    # Create directory if it doesn't exist
    os.makedirs(save_folder, exist_ok=True)

# Function to create a Folium map from filtered links
def create_folium_map(csv_file):
    """
    Reads a CSV file with link data, filters out links west of S. Halsted St.,
    and visualizes the remaining links on a Folium map.
    """
    # Load the data from the CSV file
    df = pd.read_csv(csv_file)

    # Filter out links that are west of S. Halsted St. (longitude < -87.64689)
    filtered_df = df[
        (df["start_longitude"] >= -87.64689) & (df["end_longitude"] >= -87.64689)
    ]

    # Create a map centered around Chicago
    map_center = [41.86745, -87.62041]  # Approximate center of the links
    m = folium.Map(location=map_center, zoom_start=14)

    # Add markers and lines for each filtered link
    for index, row in filtered_df.iterrows():
        # Add start point marker
        folium.Marker(
            location=[row["start_latitude"], row["start_longitude"]],
            popup=f"Start: {row['road']} ({row['tmc']})",
            icon=folium.Icon(color="green")
        ).add_to(m)

        # Add end point marker
        folium.Marker(
            location=[row["end_latitude"], row["end_longitude"]],
            popup=f"End: {row['road']} ({row['tmc']})",
            icon=folium.Icon(color="red")
        ).add_to(m)

        # Draw a line between start and end points
        folium.PolyLine(
            locations=[[row["start_latitude"], row["start_longitude"]], [row["end_latitude"], row["end_longitude"]]],
            color="blue",
            weight=2.5,
            opacity=1
        ).add_to(m)

    # Save the map to an HTML file
    m.save( os.join(save_folder, "filtered_roosevelt_rd_links_map.html") )
    print("Folium map saved as 'filtered_roosevelt_rd_links_map.html'.")

    return filtered_df  # Return filtered DataFrame for use in the next function


def visualize_speed_travel_time(travel_time_csv, filtered_df, target_date):
    """
    Reads a CSV file with speed and travel time data, filters it based on the TMC codes
    from the filtered links, and visualizes the change in speed and travel time over time
    as 2D heatmaps using matplotlib. The y-axis shows the link name as the "intersection" column,
    ordered spatially from east to west. Data is filtered for a specific target date.
    """
    # Load the travel time data
    travel_time_df = pd.read_csv(travel_time_csv)

    # Convert measurement_tstamp to datetime
    travel_time_df["measurement_tstamp"] = pd.to_datetime(travel_time_df["measurement_tstamp"])

    # Filter data for the target date
    travel_time_df = travel_time_df[travel_time_df["measurement_tstamp"].dt.date == pd.to_datetime(target_date).date()]

    # Filter travel time data to include only TMC codes from the filtered links
    filtered_tmc_codes = filtered_df["tmc"].unique()
    filtered_travel_time_df = travel_time_df[travel_time_df["tmc_code"].isin(filtered_tmc_codes)]

    # Merge with filtered_df to get direction, intersection, and start_longitude information
    filtered_travel_time_df = filtered_travel_time_df.merge(
        filtered_df[["tmc", "direction", "intersection", "start_longitude"]], 
        left_on="tmc_code", right_on="tmc", how="left"
    )

    # Extract hour from measurement_tstamp
    filtered_travel_time_df["hour"] = filtered_travel_time_df["measurement_tstamp"].dt.hour

    # Sort intersections by start_longitude (east to west)
    spatial_order = filtered_df.sort_values("start_longitude", ascending=False)["intersection"].unique()
    filtered_travel_time_df["intersection"] = pd.Categorical(
        filtered_travel_time_df["intersection"], categories=spatial_order, ordered=True
    )
    filtered_travel_time_df = filtered_travel_time_df.sort_values("intersection")

    # Create heatmaps for eastbound and westbound speed and travel time
    for direction in ["EASTBOUND", "WESTBOUND"]:
        direction_data = filtered_travel_time_df[filtered_travel_time_df["direction"] == direction]

        if not direction_data.empty:
            # Pivot data for heatmap
            speed_pivot = direction_data.pivot_table(
                index="intersection", columns="hour", values="speed", aggfunc="mean"
            )
            travel_time_pivot = direction_data.pivot_table(
                index="intersection", columns="hour", values="travel_time_seconds", aggfunc="mean"
            )

            # Plot speed heatmap
            plt.figure(figsize=(12, 6))
            plt.imshow(speed_pivot, cmap="viridis", aspect="auto")
            plt.colorbar(label="Speed (mph)")
            plt.title(f"{direction} Speed Heatmap on {target_date}")
            plt.xlabel("Hour of Day")
            plt.ylabel("Intersection (East to West)")
            plt.xticks(np.arange(0, 24, 1))
            plt.yticks(np.arange(len(speed_pivot.index)), speed_pivot.index)
            plt.savefig( os.join(save_folder, f"{direction.lower()}_speed_heatmap_{target_date}.png"), bbox_inches="tight" )
            plt.close()

            # Plot travel time heatmap
            plt.figure(figsize=(12, 6))
            plt.imshow(travel_time_pivot, cmap="plasma", aspect="auto")
            plt.colorbar(label="Travel Time (seconds)")
            plt.title(f"{direction} Travel Time Heatmap on {target_date}")
            plt.xlabel("Hour of Day")
            plt.ylabel("Intersection (East to West)")
            plt.xticks(np.arange(0, 24, 1))
            plt.yticks(np.arange(len(travel_time_pivot.index)), travel_time_pivot.index)
            plt.savefig( os.join(save_folder, f"{direction.lower()}_travel_time_heatmap_{target_date}.png"), bbox_inches="tight")
            plt.close()

# Main script
if __name__ == "__main__":
    # Paths to CSV files
    links_csv = "data/Roosevelt-202405/TMC_Identification.csv"   # Replace with your links CSV file path
    travel_time_csv = "data/Roosevelt-202405/Roosevelt-202405.csv"   # Replace with your travel time CSV file path

    # Step 1: Create Folium map and get filtered links
    filtered_links_df = create_folium_map(links_csv)

    # Step 2: Visualize speed and travel time for filtered links
    target_date = "2024-05-14"  # Replace with your target date
    visualize_speed_travel_time(travel_time_csv, filtered_links_df, target_date)