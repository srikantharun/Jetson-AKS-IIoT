# Weight sensor reading message
std_msgs/Header header

# Sensor ID and location information
string sensor_id
string shelf_id
int32 row
int32 column

# Raw sensor reading in grams
float64 raw_weight

# Calibrated weight in kilograms
float64 calibrated_weight

# Confidence value (0.0-1.0)
float64 confidence

# Status flags
bool is_calibrated
bool is_stable
bool weight_changed
