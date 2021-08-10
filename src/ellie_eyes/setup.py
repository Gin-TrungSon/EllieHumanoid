from setuptools import setup

package_name = 'ellie_eyes'
object_detection =  'ellie_eyes/object_detection_lite'
face_recognition  = 'ellie_eyes/face_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,object_detection,face_recognition],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DongTrungSon',
    maintainer_email='trungsondo68839@th-nuernberg.de',
    description='Ellie_eyes with object detection and face recognition',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ellie_eyes = ellie_eyes.ellie_eyes:main",
        ],
    },
)
