FROM ubuntu:24.04

# Install OpenMPI and OpenSSH server
RUN apt-get update && \
    apt-get install -y openmpi-bin openssh-server sudo && \
    mkdir -p /var/run/sshd

# Create mpiuser and its SSH directory
RUN useradd -m mpiuser && \
    mkdir -p /home/mpiuser/.ssh && \
    chown mpiuser:mpiuser /home/mpiuser/.ssh

# Allow mpiuser to use sudo without a password
RUN echo "mpiuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Generate SSH keys and configure passwordless SSH
RUN ssh-keygen -t rsa -N "" -f /home/mpiuser/.ssh/id_rsa && \
    cat /home/mpiuser/.ssh/id_rsa.pub >> /home/mpiuser/.ssh/authorized_keys && \
    chmod 600 /home/mpiuser/.ssh/authorized_keys && \
    chown -R mpiuser:mpiuser /home/mpiuser/.ssh

# Adjust SSH configuration to allow root login and passwordless SSH
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "PubkeyAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config

# Expose SSH port
EXPOSE 22

# Install CMake
RUN sudo apt-get update && \
    sudo apt-get install -y cmake

# Install C++ compiler
RUN sudo apt-get install -y g++

# Install Boost
RUN sudo apt-get install -y libboost-all-dev

# Install YAML-CPP
RUN sudo apt-get install -y libyaml-cpp-dev

# Install Protobuf
RUN sudo apt-get install -y protobuf-compiler libprotobuf-dev

# Start the SSH service and keep the container running
CMD ["/usr/sbin/sshd", "-D"]
