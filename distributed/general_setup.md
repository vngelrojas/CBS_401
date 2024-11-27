# Distributed Setup for MPI Cluster

This document outlines how to set up multiple servers for a distributed MPI cluster.

## Prerequisites

The following packages must be installed on all systems in the cluster:

- `libyaml-cpp-dev`
- `libboost-all-dev`
- `protobuf-compiler`
- `libprotobuf-dev`
- `openmpi-bin`

Ensure that the same versions of the above packages are installed on every system.

## Step 1: Configure Your Hosts File

To simplify communication between the machines in your cluster, you can assign hostnames to the IP addresses of each node. This eliminates the need to type in the IP addresses every time.

### Example: Master Node `/etc/hosts`

Edit the `hosts` file on the master node:

```bash
sudo nano /etc/hosts
```

Add the following entries to map the IPs to hostnames:

```plaintext
#MPI CLUSTERS
123.45.67.999 master
312.45.67.999 worker1
214.66.67.999 worker2
```

### Example: Worker Node `/etc/hosts`

Edit the `hosts` file on the worker node (e.g., `worker2`):

```bash
sudo nano /etc/hosts
```

Add the following entries:

```plaintext
#MPI CLUSTER SETUP
123.45.67.999   master
214.66.67.999   worker2
```

## Step 2: Create a New User

To keep things organized, it’s recommended to create a new user with the same username on all the machines.

### Create the User:

```bash
sudo adduser mpiuser
```

### Grant `sudo` Privileges:

```bash
sudo usermod -aG sudo mpiuser
```

## Step 3: Setting Up SSH

Machines will communicate over the network via SSH and share data via NFS. Follow the process for both the manager and worker nodes.

If the below doesnt work, you may have to manually paste keys around the servers. Regardless of what steps you take, the end goal should be that all servers should be able to ssh into each other without a password.

### Install SSH:

To install SSH on the system:

```bash
sudo apt-get install openssh-server
```

### Log in as `mpiuser`:

```bash
su - mpiuser
```

### Generate SSH Keys:

Navigate to the `~/.ssh` folder and generate an SSH key pair:

```bash
cd ~/.ssh/
ssh-keygen -t rsa
```

This will create a public-private key pair (`id_rsa` and `id_rsa.pub`).

### Add Public Key to `authorized_keys`:

Append the contents of `id_rsa.pub` to the `authorized_keys` file to allow passwordless login:

```bash
cat id_rsa.pub >> authorized_keys
```

### Copy SSH Key to Worker Nodes:

Use `ssh-copy-id` to copy the public key to the worker nodes, allowing you to SSH into them without entering a password.

For example, to copy the key to `worker2`:

```bash
ssh-copy-id worker2
```

Now, you should be able to SSH into the worker node without needing to enter the password:

```bash
ssh worker2
```

### Copy SSH Key to Manager Node from Worker Nodes:

On the worker nodes, use `ssh-copy-id` to allow passwordless SSH access to the manager node:

```bash
ssh-copy-id manager
```

Now, you can SSH into the manager node from the worker nodes without entering the password. 


## Step 4: Setting Up NFS

NFS (Network File System) will be used to share directories between the manager and worker nodes. 

### NFS Setup on Master Node

1. Install the required NFS server package:

   ```bash
   sudo apt-get install nfs-kernel-server
   ```

2. Create the directory to share (e.g., `code`):

   ```bash
   sudo mkdir /home/mpiuser/code
   ```

3. Export the directory by editing `/etc/exports`:

   ```bash
   sudo nano /etc/exports
   ```

4. Add the following entry to export the shared directory:

   ```plaintext
   /home/mpiuser/code *(rw,sync,no_root_squash,no_subtree_check)
   ```

   Alternatively, you can specify specific IP addresses instead of using `*` to allow only particular machines to access the shared folder. For example:

   ```plaintext
   /home/mpiuser/code 172.20.36.121(rw,sync,no_root_squash,no_subtree_check)
   ```

5. Apply the export settings:

   ```bash
   sudo exportfs -a
   ```

   If this doesn't work, try:

   ```bash
   sudo exportfs -a
   ```

6. Restart the NFS server if necessary:

   ```bash
   sudo service nfs-kernel-server restart
   ```

### NFS Setup on Worker Nodes

1. Install the required NFS client package:

   ```bash
   sudo apt-get install nfs-common
   ```

2. Create a directory (`cloud`) on the worker node to mount the shared folder:

   ```bash
   mkdir ~/cloud
   ```

3. Mount the shared directory from the master node:

   ```bash
   sudo mount -t nfs master:/home/mpiuser/code ~/cloud
   ```

4. To verify that the directory is mounted, use:

   ```bash
   df -h
   ```

### Make the NFS Mount Permanent

To ensure the shared directory is automatically mounted after a system reboot, add an entry to `/etc/fstab`:

```bash
sudo nano /etc/fstab
```

Add the following line:

```plaintext
#MPI CLUSTER SETUP
master:/home/mpiuser/code /home/mpiuser/cloud nfs
```

---

By following these steps, you should be able to set up a basic MPI cluster with multiple nodes communicating over SSH and sharing data via NFS.

### Clone and build the source code
1. clone the source code into the NFS directory 
2. follow the top-level README to build the distributed binary 
3. In the master run the following to test that it works
   
```bash
mpirun -np 2 -H worker1,worker2 ./CBS_distributed -i ../map_file/debug_cbs_data.yaml -o ../outputs/output.yaml
```


### Example
example.md outlines steps for working on a mock cluster via docker. The docker image sets up networking and mocks an NFS directory.