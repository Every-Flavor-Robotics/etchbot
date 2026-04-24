import math

preamble = ["G21", "G90"]
end = [""]

FEEDRATE = 12000
RESOLUTION = 2

# Create a circle of radial lines
# In millimeters
lengths = [40, 20, 10, 2.0]

# Create a list of angles
angles = [0, 15, 30, 45, 60, 75]

# Confirm that lengths and angles make sense and that there are exactly 4 quadrants
assert len(lengths) == 4

center_x = 130 / 2
center_y = 89.375 / 2

# Create a list of lines
lines = []
lines.extend(preamble)

for i in range(4):
    quadrant = 90 * i
    for angle in angles:
        # Calculate the x and y coordinates of the end of the line
        x = lengths[i] * math.cos(math.radians(angle + quadrant)) + center_x
        y = lengths[i] * math.sin(math.radians(angle + quadrant)) + center_y

        # Append the line to the list
        lines.append(f"G1 X{x:5f} Y{y:5f} F{FEEDRATE}")

        # Return to the origin
        lines.append(f"G1 X{center_x} Y{center_y} F{FEEDRATE}")

# Draw a circle of radius max(lengths) centered at the origin
# Interpolate the circle at resolution RESOLUTION
radius = max(lengths)
num_points = int(2 * math.pi * radius / RESOLUTION)
for i in range(num_points):
    angle = 360 * i / num_points
    x = radius * math.cos(math.radians(angle)) + center_x
    y = radius * math.sin(math.radians(angle)) + center_y
    lines.append(f"G1 X{x:5f} Y{y:5f} F{FEEDRATE}")

# add 0 degrees to the end
lines.append(f"G1 X{(center_x + radius):5f} Y{(center_y):5f} F{FEEDRATE}")
lines.append(f"G1 X{center_x:5f} Y{center_y:5f} F{FEEDRATE}")

# Return to the origin
lines.append(f"G1 X0 Y0 F{FEEDRATE}")

lines.extend(end)

# Write the lines to a file
with open("test_circle.optgcode", "w") as f:
    f.write("\n".join(lines))

print("File written to test_circle.optgcode")
