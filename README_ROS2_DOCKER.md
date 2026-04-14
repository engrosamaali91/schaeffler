# ROS 2 Docker Deployment

## Table of Contents

- [ROS 2 Docker Deployment](#ros-2-docker-deployment)
  - [Table of Contents](#table-of-contents)
  - [Running a Container](#running-a-container)
  - [Managing Containers](#managing-containers)
  - [ROS 2 Domain ID Configuration](#ros-2-domain-id-configuration)

---

## Running a Container

To deploy the ROS 2 package in the Docker container:

```bash
xhost +local:

docker run -it --rm --network host \
            --ipc host \
            -e DISPLAY=$DISPLAY \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            emma_in_gazebo:latest bash
```

---

## Managing Containers

**Start an existing container:**

```bash
docker start -ai <container_name>
```

**Execute a command in a running container:**

```bash
docker exec -it <container_id> bash
```

---

## ROS 2 Domain ID Configuration

> [!IMPORTANT]
> Ensure ROS 2 topic discovery between container and host by using the same `ROS_DOMAIN_ID` on both environments.

**Check current domain ID:**

```bash
echo $ROS_DOMAIN_ID
```

**Set to default (0):**

```bash
export ROS_DOMAIN_ID=0
```

**Run container with specific domain ID:**

```bash
xhost +local:
docker run -it --rm --network host -e ROS_DOMAIN_ID=51 ros:humble bash
```

> [!NOTE]
> If domain IDs differ, ROS 2 nodes will not discover each other and topics will not be visible across environments.