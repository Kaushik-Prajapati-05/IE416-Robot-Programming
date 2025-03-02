import csv
import matplotlib.pyplot as plt

def read_csv(filename):
    """Reads time and manipulability from a CSV file."""
    time_values = []
    manipulability_values = []
    
    with open(filename, "r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header
        
        for row in reader:
            time_values.append(float(row[0]))  # Time (s)
            manipulability_values.append(float(row[1]))  # Manipulability

    return time_values, manipulability_values

# Read data from two CSV files
time1, manipulability1 = read_csv("result_0_4_0.csv")  
time2, manipulability2 = read_csv("result_0_m4_0.csv")
time3, manipulability3 = read_csv("result_4_0_0.csv")  
time4, manipulability4 = read_csv("result_m4_0_0.csv") 

# Plot
plt.figure(figsize=(8, 6))
plt.plot(time1, manipulability1, label="4m front")
plt.plot(time2, manipulability2, label="4m back")
plt.plot(time3, manipulability3, label="4m left")
plt.plot(time4, manipulability4, label="4m right")
plt.xlabel("Time (s)")
plt.ylabel("Manipulability")
plt.title("Manipulability vs. Time ")
plt.xlim((0, 5))
plt.ylim((0.6, 1.75))
plt.legend()
plt.grid(True)
plt.savefig('plot.png')
plt.show()