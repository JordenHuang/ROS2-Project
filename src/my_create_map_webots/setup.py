import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'my_create_map_webots'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 套件資源索引
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        
        # 套件基本描述
        (f'share/{package_name}', ['package.xml']),

        # 啟動檔（launch）
        (f'share/{package_name}/launch', glob('launch/*.py')),

        # 世界檔（Webots world）
        (f'share/{package_name}/worlds', glob('worlds/*')),

        # URDF 模型
        (f'share/{package_name}/resource', glob('resource/*.urdf')),

        # PROTO 檔（Webots）
        (f'share/{package_name}/protos', glob('protos/*.proto')),

        # PROTO (create_icons)
        (f'share/{package_name}/protos/create_icons', glob('protos/create_icons/*')),

        # PROTO (create_textures)
        (f'share/{package_name}/protos/create_textures', glob('protos/create_textures/*')),

        (f'share/{package_name}/textures/SofaLeather', glob('textures/SofaLeather/*')),
        (f'share/{package_name}/textures/WoodFloor', glob('textures/WoodFloor/*')),

        # Config files
        (f'share/{package_name}/config', glob('config/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jordenhuang',
    maintainer_email='jorden90573@gmail.com',
    description='ROS2 package for MyCreate robot integrated with Webots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'vision_node = my_create_map_webots.vision_node:main',
            # 'MySupervisorDriver.py = my_create_map_webots.MySupervisorDriver:main',
        ],
    },
)
