import os
from glob import glob

from setuptools import setup


package_name = 'racing_navigating'


def package_files(install_dir, pattern):
    return [(os.path.join('share', package_name, install_dir), glob(pattern))]


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *package_files('config', 'config/*'),
        *package_files('map', 'map/*'),
        *package_files('launch', 'launch/*.py'),
        *package_files('launch/params/bringup', 'launch/params/bringup/*.py'),
        *package_files('launch/params/commander', 'launch/params/commander/*.py'),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='基于固定起点的 Nav2 导航与二维码任务执行节点',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fixed_start_manager = racing_navigating.fixed_start_manager:main',
            'route_executor = racing_navigating.route_executor:main',
            'scan_merger = racing_navigating.scan_merger:main',
        ],
    },
)
