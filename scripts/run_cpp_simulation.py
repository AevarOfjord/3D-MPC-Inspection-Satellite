import os
import subprocess
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description="Build and run C++ Control Simulation")
    parser.add_argument("--config", default="mission_test.yaml", help="Config file name in Cplusplus_build/config/")
    parser.add_argument("--clean", action="store_true", help="Clean build directory")
    parser.add_argument("--no-viz", action="store_true", help="Skip visualization after simulation")
    
    args = parser.parse_args()
    
    # Paths
    # Script is in scripts/, so root is one level up
    root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    cpp_dir = os.path.join(root_dir, "Cplusplus_build")
    build_dir = os.path.join(cpp_dir, "build")
    
    # Ensure build dir exists
    if not os.path.exists(build_dir):
        os.makedirs(build_dir)
        
    # Clean if requested
    if args.clean:
        print("Cleaning build directory...")
        subprocess.run("rm -rf *", cwd=build_dir, shell=True)
    
    print(f"Building in {build_dir}...")
    
    # Configure (if Makefile doesn't exist or just always to be safe? cmake .. is fast)
    # We run cmake .. to ensure new files are picked up
    try:
        subprocess.check_call(["cmake", ".."], cwd=build_dir)
    except subprocess.CalledProcessError:
        print("CMake configuration failed.")
        sys.exit(1)
        
    # Build
    try:
        subprocess.check_call(["cmake", "--build", "."], cwd=build_dir)
    except subprocess.CalledProcessError:
        print("Build failed.")
        sys.exit(1)
    
    # Run
    print(f"Running simulation with config: {args.config}")
    
    # Check if executable exists
    exe_path = os.path.join(build_dir, "sat_control")
    if not os.path.exists(exe_path):
        print(f"Executable not found at {exe_path}")
        sys.exit(1)

    try:
        # Pass the config as relative path from the build dir
        config_arg = f"../config/{args.config}"
        cmd = ["./sat_control", config_arg]
        subprocess.check_call(cmd, cwd=build_dir)
    except subprocess.CalledProcessError:
        print("Simulation runtime error.")
        sys.exit(1)
    
    print("Simulation Complete.")

    if not args.no_viz:
        print("Starting Visualization...")
        viz_script = os.path.join(root_dir, "scripts", "visualize_cpp.py")
        if os.path.exists(viz_script):
            subprocess.check_call([sys.executable, viz_script])
        else:
            print(f"Visualization script not found at {viz_script}")

if __name__ == "__main__":
    main()
