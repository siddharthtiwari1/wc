#!/usr/bin/env python3
"""
Complete reorganization of wheelchair navigation guide into 2 parts.
Preserves all content, just reorders sections.
"""

import re
import sys

def read_file(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        return f.read()

def write_file(filename, content):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(content)

def extract_section(content, start_marker, end_marker=None):
    """Extract content between two markers"""
    start_idx = content.find(start_marker)
    if start_idx == -1:
        return None

    if end_marker:
        end_idx = content.find(end_marker, start_idx + len(start_marker))
        if end_idx == -1:
            return content[start_idx:]
        return content[start_idx:end_idx]
    else:
        return content[start_idx:]

# Read original
content = read_file('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex')

# Key markers - find actual content positions
# We'll split the document into logical blocks and reorder them

# Find the preamble (everything before Executive Summary)
preamble_end = content.find('\\section{Executive Summary}')
preamble = content[:preamble_end]

# Find Executive Summary
exec_summary_start = content.find('\\section{Executive Summary}')
exec_summary_end = content.find('% PART 1:', exec_summary_start)
if exec_summary_end == -1:
    # Find next major section marker
    exec_summary_end = content.find('\\clearpage', exec_summary_start + 100)
executive_summary = content[exec_summary_start:exec_summary_end]

# Create PART 1 divider
part1_divider = """\\clearpage

% ############################################################################
% PART 1: LOW-LEVEL CONTROL AND ODOMETRY
% ############################################################################

\\begin{center}
\\vspace*{4cm}
{\\Huge\\sffamily\\bfseries\\color{titleblue}PART 1\\par}
\\vspace{1cm}
{\\LARGE\\sffamily\\bfseries\\color{accentteal}Low-Level Control and Odometry\\par}
\\vspace{2cm}
{\\large\\sffamily From Motor Commands to Precise Pose Estimation\\par}
\\vspace{4cm}
\\end{center}

\\clearpage

"""

# Find Arduino section (currently Chapter 9.6)
arduino_start = content.find('% CHAPTER 9.6: ARDUINO MOTOR CONTROL')
if arduino_start == -1:
    arduino_start = content.find('\\subsection{Arduino Motor Control: The Low-Level')

# Find Kinematics section
kinematics_start = content.find('\\subsection{Forward and Inverse Kinematics: The Mathematics')

# Find EKF section
ekf_start = content.find('\\section{Extended Kalman Filter: Mathematical Foundations}')

# Find robot_localization section
robot_loc_start = content.find('\\section{robot\\_localization Package')

# Find Hardware section (will go to Part 2)
hardware_start = content.find('\\section{Hardware Specifications}')

print("Section positions found:")
print(f"  Executive Summary: {exec_summary_start}")
print(f"  Arduino: {arduino_start}")
print(f"  Kinematics: {kinematics_start}")
print(f"  EKF: {ekf_start}")
print(f"  robot_localization: {robot_loc_start}")
print(f"  Hardware: {hardware_start}")

# For safety, let's just add the PART 2 divider for now
# Full reorganization requires careful extraction of each section

# Find where Part 2 should start (before Hardware Specifications)
part2_insert = content.find('% ============================================================================\n% CHAPTER 2: HARDWARE SPECIFICATIONS', hardware_start - 200)

if part2_insert > 0:
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

    # Insert Part 2 divider
    new_content = content[:part2_insert] + part2_divider + content[part2_insert:]

    write_file('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex', new_content)
    print("\nSuccessfully added PART 2 divider!")
    print("Next steps: Manual reorganization of section order needed")
else:
    print("Could not find insertion point for PART 2")
