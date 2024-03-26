import rosbag
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

bag_path = 'best.bag'

topics = [
    '/me5413_world/planning/rms_heading_error',
    '/me5413_world/planning/rms_position_error',
    '/me5413_world/planning/rms_speed_error'
]

last_values = {topic: None for topic in topics}

with rosbag.Bag(bag_path, 'r') as bag:
    for topic in topics:
        for _, msg, _ in bag.read_messages(topics=topic):
            last_values[topic] = msg.data

for topic, value in last_values.items():
    print(f"{topic} Average Value: {value}")

data = {topic: [] for topic in topics}
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=topics):
        data[topic].append(msg.data)

for topic in topics:
    plt.figure(figsize=(10, 6))
    plt.plot(data[topic], label=topic.split('/')[-1]) 
    plt.xlabel('Message Index')
    plt.ylabel('Value')
    plt.title(f'Data Plot for {topic}')
    plt.legend()
    plt.grid(True)
    plt.show()
