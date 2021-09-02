from setuptools import setup

package_name = 'ellie_screens'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='dongtrungson.95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_head = ellie_screens.ellie_head_monitor:main",
            "start_brust = ellie_screens.ellie_brust_monitor:main"
        ],
    },
)
