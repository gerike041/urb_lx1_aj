from setuptools import setup

package_name = 'rally_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # <<< EZ A SOR MÁSOLJA BE A LAUNCH-T >>>
        ('share/' + package_name + '/launch', ['launch/rally_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Urbán Gergely',
    maintainer_email='ugergo0928@gmail.com',
    description='Rally communication simulator (RaceControl, StageMarshal, CarTelemetry).',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'race_control = rally_comm.race_control:main',
            'stage_marshal = rally_comm.stage_marshal:main',
            'car_telemetry = rally_comm.car_telemetry:main',
        ],
    },
)
