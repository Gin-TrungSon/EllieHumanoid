from setuptools import setup
import os
from glob import glob
from setuptools import find_packages
package_name = 'ellie_arm'
sub_package_dynamixel = 'ellie_arm/dynamixel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,sub_package_dynamixel],
    data_files=[
        
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*.urdf')),
        (os.path.join('share', package_name), glob('actions/*.move')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='trungson',
    maintainer_email='trungsondo6883@th-nuernberg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [     
            "start = ellie_arm.ellie_arm:main"
        ],
    },
)
