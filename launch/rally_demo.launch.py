from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cars = [
        {'car_id': 'car_11', 'loss_prob': 0.10},
        {'car_id': 'car_21', 'loss_prob': 0.25},
        {'car_id': 'car_46', 'loss_prob': 0.05},
    ]
    car_nodes = [
        Node(package='rally_comm', executable='car_telemetry',
             name=f'car_{c["car_id"]}', parameters=[c]) for c in cars
    ]
    return LaunchDescription([
        Node(package='rally_comm', executable='race_control', name='race_control'),
        Node(package='rally_comm', executable='stage_marshal', name='marshal_ss1',
             parameters=[{'stage':'SS1'}]),
        *car_nodes
    ])
