FROM debian:buster

# Avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    python3 \
    python3-pip \
    python3-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Create directory structure
WORKDIR /app
RUN mkdir -p /app/td3learn /app/models /app/data

# Install stub TIDL and OpenCL headers for development without hardware
RUN mkdir -p /opt/tidl/api/tidl_api/inc/tidl
RUN mkdir -p /opt/opencl/api/builtins/include/CL

# Create stub TIDL header
RUN echo '// TIDL API stub header\n#ifndef TIDL_API_H\n#define TIDL_API_H\n\nextern "C" {\n  int TidlGetPreferredBatchSize(int numNetworks);\n}\n\n#endif' > /opt/tidl/api/tidl_api/inc/tidl/tidl_api.h

# Create stub OpenCL header
RUN echo '// OpenCL stub header\n#ifndef CL_HPP\n#define CL_HPP\n\nnamespace cl {\n  class Platform {};\n  class Error {};\n}\n\n#define CL_PLATFORM_NAME 0\n\n#endif' > /opt/opencl/api/builtins/include/CL/cl.hpp

# Set environment variables
ENV PYTHONPATH=/app
ENV PATH="/app/bin:${PATH}"

# Entry point
ENTRYPOINT ["/bin/bash"]
