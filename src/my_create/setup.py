import os
from glob import glob
from setuptools import setup,find_packages

package_name = 'my_create'

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
        # Config files
        (f'share/{package_name}/config', glob('config/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jordenhuang',
    maintainer_email='jorden90573@gmail.com',
    description='ROS2 package for MyCreate robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
