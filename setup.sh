# 1. Create the venv (activates upgrade pip target in makefile)
make venv

# 2. Activate the environment (impacting THIS terminal session)
source .venv311/bin/activate

# 3. Install everything (using the pip from the ACTIVATED environment)
# make install depends on venv, but since we already made it, it just runs the install steps.
make install
