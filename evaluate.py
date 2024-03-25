import rosbag
from std_msgs.msg import Float32

bag_path = 'test.bag'

topics = [
    '/me5413_world/planning/rms_heading_error',
    '/me5413_world/planning/rms_position_error',
    '/me5413_world/planning/rms_speed_error'
]

data = {topic: {'sum': 0.0, 'count': 0} for topic in topics}

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        data[topic]['sum'] += msg.data
        data[topic]['count'] += 1
        
for topic in topics:
    if data[topic]['count'] > 0:
        average = data[topic]['sum'] / data[topic]['count']
        print(f"{topic} Average: {average}")
    else:
        print(f"{topic} has no data.")
