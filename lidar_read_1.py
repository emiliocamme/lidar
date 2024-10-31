import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

# Establish connection
client = RemoteAPIClient()
sim = client.getObject('sim')

# Create figure for the polar plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')

# Wait a little to ensure the data is written in the scene
time.sleep(1)

while True:
    # Fetch the stored custom table data from the scene object
    data = sim.readCustomTableData(sim.handle_scene, "lidarData")
    
    if data:
        print("Retrieved data:", data)  # Debugging: Print the retrieved data

        distances = []
        angles = []

        # Unpack flat list into distance-angle pairs
        
        for i in range(0, len(data), 2):
            try:
                distance = data[i]
                angle = data[i + 1]
                distances.append(distance)
                angles.append(angle)
            except IndexError:
                print("Error: Incomplete data pair encountered.")

        # Debugging: Check if distances and angles are populated
        print("Distances:", distances)
        print("Angles:", angles)

        # Clear and update polar plot
        ax.clear()
        ax.scatter(angles, distances)
        plt.pause(0.05)  # Slight delay to update the plot

    else:
        print("No data found, retrying...")

    time.sleep(0.1)  # Short delay
