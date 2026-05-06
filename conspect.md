Запускаем рос-мост
source ~/vtol_ws/install/setup.bash
ros2 launch bringup flight.launch.py

Запускаем взлет и посадку по гпс
source ~/vtol_ws/install/setup.bash
ros2 run flight_manager gps_takeoff_land_node