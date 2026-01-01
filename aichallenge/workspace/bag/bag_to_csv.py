from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import pandas as pd

bag_path = 'run04'

typestore = get_typestore(Stores.ROS2_HUMBLE)

rows = []

with Reader(bag_path) as reader:
    for connection, timestamp, rawdata in reader.messages():
        t = timestamp * 1e-9  # ns -> s
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

        if connection.topic == '/control/command/control_cmd':
            rows.append({
                't': t,
                'target_speed': float(msg.longitudinal.speed),
                'target_steer': float(msg.lateral.steering_tire_angle),
                'actual_speed': None,
            })

        elif connection.topic == '/localization/kinematic_state':
            # nav_msgs/msg/Odometry
            rows.append({
                't': t,
                'target_speed': None,
                'target_steer': None,
                'actual_speed': float(msg.twist.twist.linear.x),
            })

df = pd.DataFrame(rows).sort_values('t')

# 前方補間で揃える（target/actualが別topicなので）
df[['target_speed', 'target_steer', 'actual_speed']] = df[['target_speed', 'target_steer', 'actual_speed']].ffill()

# 見やすく t=0 開始にする
df['t'] = df['t'] - df['t'].iloc[0]

out = 'run04_basic.csv'
df.to_csv(out, index=False)
print('saved:', out, 'rows:', len(df))
