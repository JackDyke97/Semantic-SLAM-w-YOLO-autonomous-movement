from setuptools import setup
import os
from glob import glob

package_name = 'semantic_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'yolo_detector = semantic_mapping.yolo_detector:main',
            'sim_fusion_node = semantic_mapping.sim_fusion_node:main',
            'fusion_node = semantic_mapping.fusion_node:main',
            'depth_from_lidar = semantic_mapping.depth_from_lidar:main',
            'random_explorer = semantic_mapping.random_explorer:main',
        ],
    },
)
