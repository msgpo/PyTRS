import matplotlib.pyplot as plt
import numpy as np

# Prepare a map to draw and a few other points.
world_map = np.ones((64, 64), np.bool)  # At first, no cell is accessible.
world_map[19:40, 19:40] = 0 # Dig a rectangle in it.
world_map[29, 24:35] = 1    # Draw three walls.
world_map[24:35, [24, 34]] = 1

youbot = [20, 23]
next_point = [23, 23]

plt.ion()
plt.figure(figsize=(15, 6))
plt.tight_layout()

# Make the first plot, just the map.
ax1 = plt.subplot(121)  # type: plt.Axes
ax1.set_aspect('equal')
plt.contour(world_map)
plt.title('Contour')

# Make the second plot, just the map.
ax2 = plt.subplot(122)  # type: plt.Axes
ax2.set_aspect('equal')
plt.imshow(world_map, origin='lower')
plt.title('Image')

plt.draw()

# Then, go back to the first plot (contour) and add the youbot as a cross.
# (Use breakpoints to see how things work, step by step.)
plt.pause(1.5)
ax1.plot(*youbot, '+', c='red', markersize=12)

# Add the next point on the other map, with a star.
plt.pause(1.5)
ax2.plot(*next_point, '*', c='green', markersize=12)

# Bring both plots to parity.
plt.pause(1.5)
ax1.plot(*next_point, '*', c='green', markersize=12)
ax2.plot(*youbot, '+', c='red', markersize=12)

# Draw the line between the two points in each window.
plt.pause(1.5)
ax1.plot(*zip(youbot, next_point), c='blue')
ax2.plot(*zip(youbot, next_point), c='blue')

plt.show(block=True)
