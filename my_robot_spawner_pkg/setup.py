import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup # Facilitates the building of packages

package_name = 'my_robot_spawner_pkg'

# Path of the current directory
cur_directory_path = os.path.abspath(os.path.dirname(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Path to the launch file      
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),

        # Path to the world file
        (os.path.join('share', package_name,'worlds/'), glob('./worlds/*')),

        # Path to the warehouse sdf file
        (os.path.join('share', package_name,'models/small_warehouse/'), glob('./models/small_warehouse/*')),

        # Path to the mobile robot sdf file
        (os.path.join('share', package_name,'models/mobile_warehouse_robot/'), glob('./models/mobile_warehouse_robot/*')),
        
        # Path to the world file (i.e. warehouse + global environment)
        (os.path.join('share', package_name,'models/'), glob('./worlds/*')),

        # ✅ Path to the Cartographer config files
        (os.path.join('share', package_name, 'config', 'cartographer'), glob('./config/cartographer/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='focalfossa',
    maintainer_email='subhransup04@gmail.com',
    description='spawn a mobile robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'spawn_demo = my_robot_spawner_pkg.spawn_demo:main'
        ],
    },
)

