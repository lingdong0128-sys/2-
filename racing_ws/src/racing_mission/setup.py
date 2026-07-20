from setuptools import setup

package_name = 'racing_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission_autorun.launch.py']),
    ],
    install_requires=['setuptools', 'pyzbar'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Mission orchestration, scene guarding, and track recording nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_state_guard = racing_mission.scene_state_guard_node:main',
            'mission_orchestrator = racing_mission.mission_orchestrator_node:main',
            'track_map_recorder = racing_mission.track_map_recorder_node:main',
        ],
    },
)
