# Defines a request to summon the robot to a specific location
# x and y represent the desired map-frame coordinate target.
float64 x
float64 y

# Target Bluetooth MAC address for final proximity homing
string ble_mac

# Mode of the request: 'static' (go to exact coords) or 'dynamic' (estimated via WiFi)
string mode
