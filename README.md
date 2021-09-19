# EllieHumanoid

EllieHumanoid is a project that provides a humanoid robot with AI functions and can communicate with ROS2.

## Features

- [x] Object detection 

- [x] Face recognition
- [x] Voice assistant
- [x] IK motion planning
- [x] Record and replay behaviors

## Installation

This install.sh file will install ROS2: foxy ,OpenCv, Dlib and other requirements. It will take a long time.

```bash
$ wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/install.sh
$ sudo chmod 755 ./install.sh
$ bash ./install.sh
```
### Installation on Raspberry (arm64/aarch64)
For some known Ubuntu issues, we have to manually create the Docker container on Raspberry
```bash
# This ensures you install the latest version of the software
$ sudo apt-get update && sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/install_arm64.sh
$ sudo chmod 755 ./install_arm64.sh
$ bash ./install_arm64.sh
```
### Using Docker
```bash
$ sudo apt-get update 
$ git clone -b devel https://github.com/Gin-TrungSon/EllieHumanoid
$ wget https://raw.githubusercontent.com/Gin-TrungSon/EllieHumanoid/devel/Dockerfile
$ docker build -t ellie:arm64 -f Dockerfile .
$ docker run -it -e DISPLAY=$DISPLAY -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix:rw -e QT_X11_NO_MITSHM=1 --privileged --name ellie_container ellie:arm64
```


## Usage
You can create a Launch file and add any module you want to use.
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ellie_eyes",
            namespace="ellie",
            executable="start",
            name="eyes"
        ),
        Node(
            package="ellie_arm",
            namespace="ellie",
            executable="start",
            name="arm"
        ),
        Node(
            package="ellie_screens",
            namespace="ellie",
            executable="start_head",
            name="head_screen"
        ),
        Node(
            package="ellie_screens",
            namespace="ellie",
            executable="start_brust",
            name="brust_screen"
        ),
        Node(
            package="ellie",
            namespace="ellie",
            executable="start",
            name="ellie"
        ),
        Node(
            package="ellie_voice",
            namespace="ellie",
            executable="start",
            name="voice"
        )
    ])

```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## Documentation (German)
[Documentation](https://github.com/Gin-TrungSon/EllieHumanoid/blob/devel/Documentation.docx)
## License
[MIT](https://choosealicense.com/licenses/mit/)

## Reference
- [Dlib](https://github.com/davisking/dlib)
- [FaceRecognition](https://github.com/ageitgey/face_recognition)
- [SpeechRecognition](/https://github.com/Uberi/speech_recognition)
- [TensorFlow](https://www.tensorflow.org/lite/guide)
- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)
