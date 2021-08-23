from setuptools import setup

package_name = 'ellie_arm'
sub_package_dynamixel = 'ellie_arm/dynamixel'
sub_package_robot = 'ellie_arm/robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,sub_package_dynamixel, sub_package_robot],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
