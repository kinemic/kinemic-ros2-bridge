# Kinemic ROS2 Bridge

## Bridge Package

The directory [`kinemic_ros_bridge`](./kinemic_ros_bridge) contains a ROS2 Python Package that forwards Events from a
[Kinemic Gesture Service](https://repo.kinemic.com/#browse/browse:kinemic-service-releases) instance to a ROS Topic.

Details about Service instance to use are currently defined in code can be configured in the
[node script `kinemic_ros_node.py`](./kinemic_ros_bridge/kinemic_ros_bridge/kinemic_ros_node.py)  
By default the Service instance is expected to run on the Docker Host executing the
[Dockerized Setup](#dockerized-setup) below.

Details about the forwarded Events can be found on the
[Kinemic Developer Portal](https://developer.kinemic.com/docs/jsonrpc/latest/)

## Dependencies

The Package uses the
[Kinemic Gesture Service Python Package](https://repo.kinemic.com/#browse/browse:pypi:kserviceconnect) to communicate
with the Service instance.

You can install this package, e.g. with `pip`, using our PyPi Repository
[https://repo.kinemic.com/repository/pypi/simple/]('https://repo.kinemic.com/repository/pypi/simple/')

```bash
python3 -m pip install -U kserviceconnect --extra-index-url 'https://repo.kinemic.com/repository/pypi/simple/'
```

## Connection Handling

The Bridge only forwards Events, and is currently not intended to handle requests or establish connections to
Kinemic Bands.

You can use the [Kinemic Remote](https://repo.kinemic.com/#browse/browse:kinemic-remote-releases) to connect to the
same Service instance the Bridge communicates with and then use it to connect Bands.

## Dockerized Setup

The [Dockerfile](./Dockerfile) in this Repo, based on the `ros:foxy` image, builds a ROS Workspace and installs the
`kinemic_ros_bridge` together with the Packages from the [ROS2 Examples Collection](https://github.com/ros2/demos.git)

This image is then used in the [Docker Compose Setup](./docker-compose.yml) to spawn both an instance of the forwarding
Bridge, and a listener that listens for events from the default topic `chatter`, that the Bridge also publishes
to by default.

To both build the image and spawn the containers, run:

```bash
docker-compose build
docker-compose up -d
```

The `listener` Container should then print the received message from the bridge:

```bash
$ docker-compose logs -f listener
Attaching to ros_bridge_listener_1
listener_1  | [INFO] [1665044762.104814454] [listener]: I heard: [{"type": "Heartbeat", "parameters": {"bands": ["e4:bc:de:23:da:97"], "timestamp": 1665044762075}}]
listener_1  | [INFO] [1665044764.078976313] [listener]: I heard: [{"type": "Heartbeat", "parameters": {"bands": ["e4:bc:de:23:da:97"], "timestamp": 1665044764076}}]
listener_1  | [INFO] [1665044766.080378131] [listener]: I heard: [{"type": "Heartbeat", "parameters": {"bands": ["e4:bc:de:23:da:97"], "timestamp": 1665044766077}}]
```
