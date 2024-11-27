# MPI Distributed Setup

This doc provides the steps to set up and run a distributed MPI example using Docker containers.

## Prerequisites
Make sure you have Docker installed and running on your system.

## Steps

### 1. Navigate to the Directory
First, navigate to the directory containing this README file:

```bash
cd /path/to/directory
```

### 2. Create Docker Network
Create a custom Docker network to enable communication between the containers:

```bash
docker network create --driver bridge mpi_network
```

### 3. Build Docker Image
Build the Docker image from the `Dockerfile`:

```bash
docker build -t openmpi .
```

### 4. Run Docker Containers
Run the master and worker containers, mounting the current directory into each container for code access:

```bash
docker run -v "$(pwd)/..:/home/mpiuser/code" -d --name mpi-master --network mpi_network openmpi
docker run -v "$(pwd)/..:/home/mpiuser/code" -d --name mpi-worker1 --network mpi_network openmpi
docker run -v "$(pwd)/..:/home/mpiuser/code" -d --name mpi-worker2 --network mpi_network openmpi
```

### 5. Access the Master Container
Enter the `mpi-master` container to perform necessary setup:

```bash
docker exec -it mpi-master /bin/bash
```

### 6. Log into the `mpiuser`
Switch to the `mpiuser` account inside the container:

```bash
su - mpiuser
```

### 7. Set Up the Build Directory
Navigate to the `code` directory, remove any existing build directory, and create a new one:

```bash
cd code
rm -rf build
```

### 8. Build the Distributed Binary
Follow the instructions in the top-level README to build the distributed binary.

### 9. Run the MPI Application
In the build directory, run the MPI application with the following command:

```bash
mpirun -np 2 -H mpi-worker1,mpi-worker2 ./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```

This will run the distributed application across the two worker containers and generate the output in the specified file.
