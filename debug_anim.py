import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path
import os

print("Testing animation save...")
fig, ax = plt.subplots()
(line,) = ax.plot([], [])
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)


def update(frame):
    line.set_data([0, frame], [0, frame])
    return (line,)


anim = animation.FuncAnimation(fig, update, frames=10)
# Ensure directory exists (it should)
output_dir = Path("Data/Simulation/16-01-2026_15-56-08")
if not output_dir.exists():
    print(f"Directory {output_dir} does not exist!")
    exit(1)

output_path = output_dir / "test_anim.mp4"

print(f"Saving to {output_path.absolute()}")
try:
    anim.save(str(output_path), writer="ffmpeg", fps=10)
    print("Success!")
except Exception as e:
    print(f"Failed: {e}")
