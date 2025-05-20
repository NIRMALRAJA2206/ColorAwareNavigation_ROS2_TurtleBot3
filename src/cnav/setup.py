from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'cnav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch') , glob('launch/*')),
        (os.path.join('share',package_name,'config') , glob('config/*')),
        (os.path.join('share',package_name,'world/maze') , glob('world/maze/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nirmalraja',
    maintainer_email='nirmalraja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'occupancy_grid_pub = cnav.occupancy_grid_pub:main',
            'sdf_spawner = cnav.spawn_entity:main',
            'msg = cnav.msg:main',
            'maze_task = cnav.maze_task:main',
            'maze_solver = cnav.maze_solver:main',
        ],
    },
)
