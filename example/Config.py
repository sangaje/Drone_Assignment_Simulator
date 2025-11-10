from dronesim.energy import WattHour
from dronesim.unit import KilometersPerHour, Minute, Watt

# File paths for data input and example output
DATA_CSV_FILE = "./train.csv"
EXPECT_CSV_FILE = "./train.csv"
CLUSTER_DATA_FILE = "./train.csv"

# DATA_CSV_FILE = "./test.csv"
# EXPECT_CSV_FILE = "./Sample_Submission.csv"
# CLUSTER_DATA_FILE = "./train.csv"

# Simulation Configuration
N_CLUSTERS = 10
WAITING_TIME = Minute(1)
DT = Minute(0.5)
DRONE_COUNT = 200

J = 1
BATCH_SIZE = 100

# Drone Configuration
DRONE_VELOCITY = KilometersPerHour(60)

# Battery Configuration
BATTERY_CAPACITY = WattHour(500)
BATTERY_CURRENT = WattHour(500)
OPERATIONAL_BATTERY_PERCENTAEG = 20

# Power & Energy Consumption (Watt)
IDLE_POWER = Watt(10)
VTOL_POWER = Watt(500)
TRAINSIT_POWER = Watt(500)
PER_PACAGE_POWER = Watt(100)

# Not Used
TASK_QUEUE_PER_DRONE=2
DELVIERYS_PER_CHARGE=2
