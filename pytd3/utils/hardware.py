"""
Hardware detection and management utilities for TD3.
"""
import os
import subprocess
import platform
import logging
import re

logger = logging.getLogger(__name__)

def is_tidl_available():
    """
    Check if TIDL (TI Deep Learning) is available.
    
    Returns:
        True if TIDL is available
    """
    # Check for TIDL libraries
    tidl_paths = [
        "/usr/share/ti/tidl",
        "/usr/local/share/ti/tidl"
    ]
    
    for path in tidl_paths:
        if os.path.exists(path):
            logger.info(f"TIDL detected at {path}")
            return True
    
    logger.info("TIDL not detected")
    return False

def is_opencl_available():
    """
    Check if OpenCL is available.
    
    Returns:
        True if OpenCL is available
    """
    # Check for TI OpenCL libraries
    opencl_paths = [
        "/usr/lib/libTIOpenCL.so",
        "/usr/lib/arm-linux-gnueabihf/libTIOpenCL.so"
    ]
    
    for path in opencl_paths:
        if os.path.exists(path):
            logger.info(f"TI OpenCL detected at {path}")
            return True
    
    logger.info("TI OpenCL not detected")
    return False

def get_hardware_info():
    """
    Get hardware information about the system.
    
    Returns:
        Dictionary with hardware information
    """
    info = {
        "platform": platform.platform(),
        "architecture": platform.machine(),
        "processor": platform.processor() or "Unknown",
        "python_version": platform.python_version(),
        "tidl_available": is_tidl_available(),
        "opencl_available": is_opencl_available()
    }
    
    # Try to get more detailed CPU info on Linux
    if platform.system() == "Linux":
        try:
            with open("/proc/cpuinfo", "r") as f:
                cpu_info = f.read()
            
            # Try to extract CPU model
            model_match = re.search(r"model name\s*:\s*(.*)", cpu_info)
            if model_match:
                info["cpu_model"] = model_match.group(1)
            
            # Try to get memory info
            with open("/proc/meminfo", "r") as f:
                mem_info = f.read()
            
            # Extract total memory
            mem_match = re.search(r"MemTotal:\s*(\d+)", mem_info)
            if mem_match:
                info["memory_kb"] = int(mem_match.group(1))
                info["memory_mb"] = info["memory_kb"] // 1024
        
        except Exception as e:
            logger.warning(f"Error getting detailed hardware info: {e}")
    
    return info

def get_beaglebone_model():
    """
    Detect BeagleBone model.
    
    Returns:
        Model name or None if not a BeagleBone
    """
    if not os.path.exists("/proc/device-tree/model"):
        return None
    
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().strip('\0')
        
        if "BeagleBone" in model:
            return model
        return None
    except Exception as e:
        logger.warning(f"Error detecting BeagleBone model: {e}")
        return None

def detect_hardware_accelerators():
    """
    Detect available hardware accelerators.
    
    Returns:
        Dictionary with accelerator information
    """
    accelerators = {
        "dsp": {
            "available": False,
            "count": 0
        },
        "eve": {
            "available": False,
            "count": 0
        },
        "gpu": {
            "available": False,
            "type": None
        }
    }
    
    # Check for DSP cores
    if os.path.exists("/dev/dsp"):
        accelerators["dsp"]["available"] = True
        
        # Count DSP cores
        dsp_count = 0
        for i in range(8):  # Check up to 8 potential DSP cores
            if os.path.exists(f"/dev/dsp{i}"):
                dsp_count += 1
        
        accelerators["dsp"]["count"] = max(1, dsp_count)
    
    # Check for EVE cores (specific to TI processors)
    beaglebone_model = get_beaglebone_model()
    if beaglebone_model and "AI" in beaglebone_model:
        # BeagleBone AI has 4 EVE cores
        accelerators["eve"]["available"] = True
        accelerators["eve"]["count"] = 4
    
    # Check for GPU acceleration via OpenCL
    if is_opencl_available():
        accelerators["gpu"]["available"] = True
        accelerators["gpu"]["type"] = "PowerVR SGX"
    
    return accelerators

def print_hardware_summary():
    """Print summary of available hardware"""
    info = get_hardware_info()
    accelerators = detect_hardware_accelerators()
    
    print("\n=== Hardware Summary ===")
    print(f"Platform:     {info['platform']}")
    print(f"Architecture: {info['architecture']}")
    
    if "cpu_model" in info:
        print(f"CPU:         {info['cpu_model']}")
    else:
        print(f"Processor:   {info['processor']}")
    
    if "memory_mb" in info:
        print(f"Memory:      {info['memory_mb']} MB")
    
    print("\nAccelerators:")
    
    # DSP
    if accelerators["dsp"]["available"]:
        print(f"- DSP:       {accelerators['dsp']['count']} cores available")
    else:
        print("- DSP:       Not available")
    
    # EVE
    if accelerators["eve"]["available"]:
        print(f"- EVE:       {accelerators['eve']['count']} cores available")
    else:
        print("- EVE:       Not available")
    
    # GPU
    if accelerators["gpu"]["available"]:
        print(f"- GPU:       {accelerators['gpu']['type']}")
    else:
        print("- GPU:       Not available")
    
    print("\nAcceleration Libraries:")
    print(f"- TIDL:      {'Available' if info['tidl_available'] else 'Not available'}")
    print(f"- OpenCL:    {'Available' if info['opencl_available'] else 'Not available'}")
    
    print("\nBeagleBone Info:")
    model = get_beaglebone_model()
    if model:
        print(f"- Model:     {model}")
    else:
        print("- Not running on a BeagleBone")
    
    print("")
