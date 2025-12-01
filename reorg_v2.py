#!/usr/bin/env python3
"""
Reorganize LaTeX document using exact line numbers.
Safe approach: extract by line ranges, reassemble in new order.
"""

def read_lines(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        return f.readlines()

def write_lines(filename, lines):
    with open(filename, 'w', encoding='utf-8') as f:
        f.writelines(lines)

print("="*70)
print("REORGANIZING WHEELCHAIR NAVIGATION GUIDE")
print("="*70)

# Read all lines
lines = read_lines('2D_LIDAR_SLAM_COMPLETE_GUIDE_BACKUP.tex')
print(f"\nTotal lines: {len(lines)}")

# Define section boundaries (line numbers are 0-indexed in Python, but grep shows 1-indexed)
# Subtract 1 from grep line numbers

sections = {
    'preamble': (0, 162),  # Start to before Executive Summary
    'exec_summary': (162, 240),  # Executive Summary to before Arduino
    'arduino_header_old': (240, 279),  # Old Arduino section header (will skip this - it's duplicate)
    'slam_old': (278, 816),  # Old SLAM content (What is SLAM? subsections)
    'hardware': (816, 915),
    'v14pro': (915, 1076),
    'deployment': (1076, 1207),
    'troubleshooting': (1207, 1273),
    'benchmarks': (1273, 1361),
    'lessons': (1361, 1457),
    'conclusion': (1457, 1520),
    'ekf': (1520, 1832),
    'robot_loc': (1832, 2506),  # Up to before Arduino subsection
    'arduino_subsection': (2506, 3177),  # Arduino content (as subsection under robot_loc)
    'kinematics': (3177, 3628),  # Kinematics (as subsection)
    'pipeline': (3628, 4042),
    'slam_toolbox': (4042, 4524),
    'hector_vs': (4524, 4725),
    'launch_file': (4725, 5161),
    'final_summary': (5161, len(lines)),
}

print("\nSection ranges:")
for name, (start, end) in sections.items():
    print(f"  {name:20s}: lines {start+1:5d} to {end:5d} ({end-start:5d} lines)")

# Extract sections
extracted = {}
for name, (start, end) in sections.items():
    extracted[name] = lines[start:end]
    print(f"Extracted {name}: {len(extracted[name])} lines")

# Create PART dividers
part1_divider = [
    "\\clearpage\n",
    "\n",
    "% ############################################################################\n",
    "% PART 1: LOW-LEVEL CONTROL AND ODOMETRY\n",
    "% ############################################################################\n",
    "\n",
    "\\begin{center}\n",
    "\\vspace*{4cm}\n",
    "{\\Huge\\sffamily\\bfseries\\color{titleblue}PART 1\\par}\n",
    "\\vspace{1cm}\n",
    "{\\LARGE\\sffamily\\bfseries\\color{accentteal}Low-Level Control and Odometry\\par}\n",
    "\\vspace{2cm}\n",
    "{\\large\\sffamily From Motor Commands to Precise Pose Estimation\\par}\n",
    "\\vspace{4cm}\n",
    "\\end{center}\n",
    "\n",
    "\\clearpage\n",
    "\n",
]

part2_divider = [
    "\\clearpage\n",
    "\n",
    "% ############################################################################\n",
    "% PART 2: MAPPING, LOCALIZATION AND SLAM\n",
    "% ############################################################################\n",
    "\n",
    "\\begin{center}\n",
    "\\vspace*{4cm}\n",
    "{\\Huge\\sffamily\\bfseries\\color{titleblue}PART 2\\par}\n",
    "\\vspace{1cm}\n",
    "{\\LARGE\\sffamily\\bfseries\\color{accentteal}Mapping, Localization and SLAM\\par}\n",
    "\\vspace{2cm}\n",
    "{\\large\\sffamily From Sensor Data to Accurate Maps\\par}\n",
    "\\vspace{4cm}\n",
    "\\end{center}\n",
    "\n",
    "\\clearpage\n",
    "\n",
]

# Build new document
new_lines = []

# 1. Preamble
new_lines.extend(extracted['preamble'])

# 2. Executive Summary
new_lines.extend(extracted['exec_summary'])

# 3. PART 1 divider
new_lines.extend(part1_divider)

# 4. Arduino (promote from subsection to section)
arduino_lines = extracted['arduino_subsection'].copy()
# Change subsection to section
for i, line in enumerate(arduino_lines):
    if '\\subsection{Arduino Motor Control' in line:
        arduino_lines[i] = line.replace('\\subsection{', '\\section{')
    elif '\\subsubsection{' in line:
        arduino_lines[i] = line.replace('\\subsubsection{', '\\subsection{')
    elif '% CHAPTER 9.6:' in line:
        arduino_lines[i] = '% ============================================================================\n'
        arduino_lines.insert(i+1, '% CHAPTER 1: ARDUINO MOTOR CONTROL\n')
        arduino_lines.insert(i+2, '% ============================================================================\n')
        break
new_lines.extend(arduino_lines)

# 5. Kinematics (promote from subsection to section)
kinematics_lines = extracted['kinematics'].copy()
for i, line in enumerate(kinematics_lines):
    if '\\subsection{Forward and Inverse Kinematics' in line:
        kinematics_lines.insert(i, '\n% ============================================================================\n')
        kinematics_lines.insert(i+1, '% CHAPTER 2: FORWARD AND INVERSE KINEMATICS\n')
        kinematics_lines.insert(i+2, '% ============================================================================\n')
        kinematics_lines[i+3] = line.replace('\\subsection{', '\\section{')
        break
    elif '\\subsubsection{' in line:
        kinematics_lines[i] = line.replace('\\subsubsection{', '\\subsection{')
new_lines.extend(kinematics_lines)

# 6. EKF (Chapter 3)
new_lines.extend(extracted['ekf'])

# 7. robot_localization (Chapter 4)
new_lines.extend(extracted['robot_loc'])

# 8. PART 2 divider
new_lines.extend(part2_divider)

# 9. SLAM Theory (Chapter 5) - promote subsections to section
slam_lines = extracted['slam_old'].copy()
# Add section header
slam_header = [
    "% ============================================================================\n",
    "% CHAPTER 5: SLAM THEORETICAL FOUNDATIONS\n",
    "% ============================================================================\n",
    "\\section{Theoretical Foundations of 2D LiDAR SLAM}\n",
    "\n",
]
new_lines.extend(slam_header)
new_lines.extend(slam_lines)

# 10. Hardware (Chapter 6)
new_lines.extend(extracted['hardware'])

# 11. v14_pro (Chapter 7)
new_lines.extend(extracted['v14pro'])

# 12. Pipeline (Chapter 8)
new_lines.extend(extracted['pipeline'])

# 13. slam_toolbox (Chapter 9)
new_lines.extend(extracted['slam_toolbox'])

# 14. Hector vs (Chapter 10)
new_lines.extend(extracted['hector_vs'])

# 15. Launch File (Chapter 11)
new_lines.extend(extracted['launch_file'])

# 16. Deployment (Chapter 12)
new_lines.extend(extracted['deployment'])

# 17. Troubleshooting (Chapter 13)
new_lines.extend(extracted['troubleshooting'])

# 18. Benchmarks (Chapter 14)
new_lines.extend(extracted['benchmarks'])

# 19. Lessons (Chapter 15)
new_lines.extend(extracted['lessons'])

# 20. Final Summary (Chapter 16)
new_lines.extend(extracted['final_summary'])

# 21. End document
if not new_lines[-1].strip().endswith('\\end{document}'):
    new_lines.append('\n\\end{document}\n')

print(f"\nNew document: {len(new_lines)} lines")
print(f"Original: {len(lines)} lines")
print(f"Difference: {len(new_lines) - len(lines)} lines (expected: ~40 for dividers)")

# Write
write_lines('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex', new_lines)

print("\n" + "="*70)
print("REORGANIZATION COMPLETE!")
print("="*70)
print("\nNew structure:")
print("  Executive Summary")
print("  PART 1: Low-Level Control and Odometry")
print("    Chapter 1: Arduino Motor Control")
print("    Chapter 2: Forward and Inverse Kinematics")
print("    Chapter 3: Extended Kalman Filter")
print("    Chapter 4: robot_localization Package")
print("  PART 2: Mapping, Localization and SLAM")
print("    Chapter 5: SLAM Theoretical Foundations")
print("    Chapter 6: Hardware Specifications")
print("    Chapter 7: v14_pro Configuration")
print("    Chapter 8: Complete Data Pipeline")
print("    Chapter 9: slam_toolbox Internals")
print("    Chapter 10: Hector SLAM vs slam_toolbox")
print("    Chapter 11: Launch File Analysis")
print("    Chapter 12: Deployment Guide")
print("    Chapter 13: Troubleshooting")
print("    Chapter 14: Performance Benchmarks")
print("    Chapter 15: Lessons Learned")
print("    Chapter 16: Final Summary")
