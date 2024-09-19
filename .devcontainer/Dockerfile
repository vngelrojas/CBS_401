FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="3.22.2"

# Optionally install the cmake for vcpkg 
COPY ./reinstall-cmake.sh /tmp/


RUN apt-get update && apt-get install -y dos2unix

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
    dos2unix /tmp/reinstall-cmake.sh && chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi && \
    rm -f /tmp/reinstall-cmake.sh


# Clone and install ParlayLib
RUN git clone https://github.com/cmuparlay/parlaylib.git /tmp/parlaylib \
    && cd /tmp/parlaylib \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && cd / \
    && rm -rf /tmp/parlaylib

# Boost and yaml-cpp
RUN sudo apt-get update && sudo apt-get install -y libboost-all-dev
RUN git clone https://github.com/jbeder/yaml-cpp.git /tmp/yaml-cpp \
    && cd /tmp/yaml-cpp \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && cd / \
    && rm -rf /tmp/yaml-cpp

#Google Test
RUN git clone https://github.com/google/googletest.git /tmp/googletest \
    && cd /tmp/googletest \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && cd / \
    && rm -rf /tmp/googletest

# Google Benchmark
RUN git clone https://github.com/google/benchmark.git /tmp/benchmark \
    && cd /tmp/benchmark \
    && git clone https://github.com/google/googletest.git /tmp/benchmark/googletest \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && cd / \
    && rm -rf /tmp/benchmark

# Python and env thing
RUN sudo apt-get update && sudo apt-get install -y python3 python3-pip
RUN python3 --version && pip3 --version
RUN sudo apt install -y python3.12-venv
