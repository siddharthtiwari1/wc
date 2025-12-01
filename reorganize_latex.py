#!/usr/bin/env python3
"""
Reorganize 2D_LIDAR_SLAM_COMPLETE_GUIDE.tex into two parts:
Part 1: Low-Level Control and Odometry
Part 2: Mapping, Localization and SLAM
"""

import re

# Read the original file
with open('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex', 'r', encoding='utf-8') as f:
    content = f.read()

# Split into lines for easier processing
lines = content.split('\n')

# Find key section markers
markers = {}
for i, line in enumerate(lines):
    if '% CHAPTER 9.6: ARDUINO MOTOR CONTROL' in line:
        markers['arduino_start'] = i
    elif '\\subsection{Forward and Inverse Kinematics: The Mathematics' in line:
        markers['kinematics_start'] = i
    elif '\\section{Extended Kalman Filter: Mathematical Foundations}' in line:
        markers['ekf_start'] = i
    elif '\\section{robot\\_localization Package' in line:
        markers['robot_loc_start'] = i
    elif '\\section{Theoretical Foundations of 2D LiDAR SLAM}' in line:
        markers['slam_theory_start'] = i
    elif '\\section{Hardware Specifications}' in line:
        markers['hardware_start'] = i
    elif '\\section{Complete Data Pipeline' in line:
        markers['pipeline_start'] = i
    elif '\\section{slam\\_toolbox Internals' in line:
        markers['slam_toolbox_start'] = i

print("Found markers:")
for key, value in sorted(markers.items(), key=lambda x: x[1]):
    print(f"  {key}: line {value}")

# Extract sections (we'll reorganize by finding section boundaries)
# For now, let's just add the PART 2 divider

# Find where to insert PART 2 (before SLAM theory section)
part2_insert_line = markers.get('slam_theory_start', 0)

if part2_insert_line > 0:
    # Insert PART 2 divider
    part2_divider = """
% ############################################################################
% PART 2: MAPPING, LOCALIZATION AND SLAM
% ############################################################################

\\begin{center}
\\vspace*{4cm}
{\\Huge\\sffamily\\bfseries\\color{titleblue}PART 2\\par}
\\vspace{1cm}
{\\LARGE\\sffamily\\bfseries\\color{accentteal}Mapping, Localization and SLAM\\par}
\\vspace{2cm}
{\\large\\sffamily From Sensor Data to Accurate Maps\\par}
\\vspace{4cm}
\\end{center}

\\clearpage

"""

    # Insert the divider
    lines.insert(part2_insert_line - 3, part2_divider)  # Insert before the comment line

    print(f"\nInserted PART 2 divider before line {part2_insert_line}")

    # Write back
    with open('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex', 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))

    print("File updated successfully!")
else:
    print("Could not find SLAM theory section marker")
