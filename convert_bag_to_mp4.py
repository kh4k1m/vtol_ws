import sqlite3
import cv2
import numpy as np
import sys
import os
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bag_file = '/home/nvidia/vtol_ws/flight_video_bag_20260324-190140/flight_video_bag_20260324-190140_0.db3'
output_file = '/home/nvidia/vtol_ws/flight_video3.mp4'

print(f"Opening bag file: {bag_file}")
conn = sqlite3.connect(bag_file)
cursor = conn.cursor()

# Get topic ID for /camera/image_raw
cursor.execute("SELECT id FROM topics WHERE name = '/camera/image_raw'")
topic_id_row = cursor.fetchone()
if not topic_id_row:
    print("Topic /camera/image_raw not found in the bag.")
    sys.exit(1)
topic_id = topic_id_row[0]

# Count total messages for progress bar
cursor.execute("SELECT count(*) FROM messages WHERE topic_id = ?", (topic_id,))
total_frames = cursor.fetchone()[0]
print(f"Found {total_frames} frames to process.")

cursor.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topic_id,))

bridge = CvBridge()
out = None

print("Starting conversion to MP4...")
count = 0
for row in cursor:
    # Deserialize the ROS 2 message
    msg = deserialize_message(row[0], Image)
    
    # Convert to OpenCV image
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    if out is None:
        height, width, layers = cv_img.shape
        print(f"Video resolution: {width}x{height}")
        # Use mp4v codec
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_file, fourcc, 30.0, (width, height))
        
    out.write(cv_img)
    count += 1
    if count % 100 == 0:
        print(f"Processed {count}/{total_frames} frames ({(count/total_frames)*100:.1f}%)")

if out:
    out.release()
conn.close()
print(f"Done! Video saved to {output_file}")
