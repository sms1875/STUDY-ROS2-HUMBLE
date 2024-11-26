from setuptools import find_packages, setup
from glob import glob

package_name = 'ptp_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='67058185+sms1875@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptp_move_node = ptp_move.ptp_move_node:main',
            'ptp_move_input_node = ptp_move.ptp_move_input_node:main',
            'ptp_move_setpos_node = ptp_move.ptp_move_setpos_node:main',
            'dobot_multi_control_pkg_node = ptp_move.dobot_multi_control_pkg_node:main',
            'object_detection_node = ptp_move.object_detection_node:main',
            'number_detection_node= ptp_move.number_detection_node:main'
        ],
    },
)
