import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read routing txt file
with open(sys.argv[1], 'r') as f:
    lines = f.readlines()[4:]

# Parse routing data
paths = {}
current_path = None
for line in lines:
    line = line.strip()
    if not line:
        continue
    if line.startswith('net') or line.startswith('Net') or line.startswith('tc'):
        current_path = []
        paths[line] = current_path
    else:
        x, y, z = map(int, line.split())
        current_path.append((x, y, z))

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# ========================================================
# Key Setting 1: Disable Matplotlib's automatic Z-order calculation
# ========================================================
ax.computed_zorder = False

# Set a fixed viewing angle
elev = 30   # Elevation
azim = -60  # Azimuth
ax.view_init(elev=elev, azim=azim)

# Calculate the camera view's normal vector to determine depth
elev_rad = np.radians(elev)
azim_rad = np.radians(azim)
cam_vec = np.array([
    np.cos(elev_rad) * np.cos(azim_rad),
    np.cos(elev_rad) * np.sin(azim_rad),
    np.sin(elev_rad)
])

# ========================================================
# Key Setting 2: Collect all graphic elements and calculate their center points
# ========================================================
elements = []
prop_cycle = plt.rcParams['axes.prop_cycle']
colors = prop_cycle.by_key()['color']
color_idx = 0

for name, path in paths.items():
    if not path or len(path) < 2:
        continue
        
    xs, ys, zs = zip(*path)
    c = colors[color_idx % len(colors)]
    color_idx += 1
    
    # Collect each line segment
    for i in range(len(xs) - 1):
        x_seg = [xs[i], xs[i+1]]
        y_seg = [ys[i], ys[i+1]]
        z_seg = [zs[i], zs[i+1]]
        # Calculate the 3D midpoint coordinates of the segment
        center = np.array([np.mean(x_seg), np.mean(y_seg), np.mean(z_seg)])
        elements.append({
            'type': 'line',
            'x': x_seg, 'y': y_seg, 'z': z_seg,
            'color': c,
            'label': name if i == 0 else "", # Add label only for the first segment
            'center': center
        })
        
    # Collect start and end points (Pins)
    for idx in [0, -1]:
        center = np.array([xs[idx], ys[idx], zs[idx]])
        elements.append({
            'type': 'point',
            'x': xs[idx], 'y': ys[idx], 'z': zs[idx],
            'color': c,
            'label': "",
            'center': center
        })

# ========================================================
# Key Setting 3: Calculate depth and reassign Z-order
# ========================================================
# Dot product of center point and camera vector; smaller values mean further from the camera
for el in elements:
    el['depth'] = np.dot(el['center'], cam_vec)

# Sort by depth in ascending order (from furthest to closest)
elements.sort(key=lambda x: x['depth'])

# Assign increasing zorder starting from 0 based on the sorted order (drawn later = top layer)
for z_idx, el in enumerate(elements):
    if el['type'] == 'line':
        ax.plot(el['x'], el['y'], el['z'], color=el['color'], 
                label=el['label'], zorder=z_idx)
    elif el['type'] == 'point':
        # depthshade must be False since we manually took over depth rendering
        ax.scatter([el['x']], [el['y']], [el['z']], color=el['color'], 
                   marker='o', depthshade=False, zorder=z_idx)

# Set axis labels and limits
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_zlim(0, None)

# Set legend (manually filter out empty labels to avoid blank rows)
handles, labels = ax.get_legend_handles_labels()
valid_handles_labels = [(h, l) for h, l in zip(handles, labels) if l]
if valid_handles_labels:
    handles, labels = zip(*valid_handles_labels)
    plt.legend(handles, labels, fontsize=6, loc='upper left', bbox_to_anchor=(1.05, 1))

fig.tight_layout()
ax.tick_params(axis='both', which='major', labelsize=10)

# Save high-resolution image
plt.savefig("path.png", dpi=600, bbox_inches='tight')
plt.show()