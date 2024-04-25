import subprocess
import time

# Number of runs
num_runs = 10

# List to store execution times
execution_times = []

for i in range(num_runs):
    # Generate input file using your Python script
    # Call your Python script to generate the graph file (replace 'generate_graph.py' with your actual script)
    subprocess.run(['python', 'Graph_gen.py'])

    # Record start time
    start_time = time.time()

    # Call your C++ program
    subprocess.run(['./dijkstra_bas_npq'])

    # Record end time
    end_time = time.time()

    # Calculate and store execution time
    execution_time = end_time - start_time
    execution_times.append(execution_time)

# Compute average execution time
average_execution_time = sum(execution_times) / num_runs
print("Average Execution Time:", average_execution_time)
