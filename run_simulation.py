import os
import sys
import subprocess

def main():
    """
    Wrapper script to run the C++ simulation helper located in scripts/run_cpp_simulation.py
    """
    script_path = os.path.join(os.path.dirname(__file__), "scripts", "run_cpp_simulation.py")
    
    if os.path.exists(script_path):
        # Forward all arguments to the actual script
        cmd = [sys.executable, script_path] + sys.argv[1:]
        subprocess.call(cmd)
    else:
        print(f"Error: Could not find simulation script at {script_path}")
        sys.exit(1)

if __name__ == "__main__":
    main()
