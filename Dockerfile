# Use Ubuntu 22.04 ARM64 for Apple Silicon compatibility
FROM ubuntu:22.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    can-utils \
    iproute2 \
    kmod \
    libeigen3-dev \
    cppcheck \
    gdb \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python CAN library
RUN pip3 install python-can

# Install FreeRTOS POSIX port
# Clone FreeRTOS kernel
RUN git clone --depth 1 --branch V10.5.1 https://github.com/FreeRTOS/FreeRTOS-Kernel.git /opt/FreeRTOS-Kernel

# Set up FreeRTOS library installation
WORKDIR /opt/FreeRTOS-Kernel
RUN mkdir -p /usr/include/freertos && \
    cp -r include/* /usr/include/freertos/ && \
    cp -r portable/ThirdParty/GCC/Posix/* /usr/include/freertos/ && \
    cp -r portable/ThirdParty/GCC/Posix/utils/* /usr/include/freertos/

# Create FreeRTOS library build directory
RUN mkdir -p build && cd build && \
    cmake -DFREERTOS_PORT=POSIX .. && \
    make && \
    cp libfreertos.a /usr/lib/ || true

# Set working directory for project
WORKDIR /workspace

# Copy project files
COPY . .

# Set up virtual CAN interface on container start
RUN echo '#!/bin/bash\n\
modprobe vcan || true\n\
ip link add dev vcan0 type vcan || true\n\
ip link set up vcan0 || true\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
