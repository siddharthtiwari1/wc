#!/usr/bin/env python3
"""
Complete reorganization of wheelchair SLAM guide.
Works on backup, creates new file with proper structure.
"""

import re

def read_file(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        return f.read()

def write_file(filename, content):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write(content)

def find_section_boundaries(content, section_markers):
    """Find start and end positions of sections"""
    sections = {}
    lines = content.split('\n')

    for i, line in enumerate(lines):
        for name, patterns in section_markers.items():
            for pattern in patterns:
                if pattern in line:
                    if name not in sections:
                        sections[name] = {'start_line': i, 'start_pos': None}
                    break

    # Convert line numbers to character positions
    pos = 0
    line_positions = [0]
    for line in lines:
        pos += len(line) + 1  # +1 for newline
        line_positions.append(pos)

    for name, info in sections.items():
        if 'start_line' in info:
            info['start_pos'] = line_positions[info['start_line']]

    return sections

print("=" * 70)
print("WHEELCHAIR NAVIGATION GUIDE - COMPLETE REORGANIZATION")
print("=" * 70)

# Read the backup file
content = read_file('2D_LIDAR_SLAM_COMPLETE_GUIDE_BACKUP.tex')
print(f"\nOriginal file size: {len(content)} characters")

# Define section markers with multiple possible patterns
section_markers = {
    'preamble_end': ['\\section{Executive Summary}'],
    'executive_summary_end': ['\\clearpage', '% PART 1:', '% CHAPTER 1:'],
    'arduino_start': ['% CHAPTER 9.6: ARDUINO', '\\subsection{Arduino Motor Control: The Low-Level'],
    'arduino_end': ['\\subsection{Forward and Inverse Kinematics'],
    'kinematics_start': ['\\subsection{Forward and Inverse Kinematics: The Mathematics'],
    'kinematics_end': ['\\clearpage', '\\section{Complete Data Pipeline'],
    'ekf_start': ['\\section{Extended Kalman Filter: Mathematical Foundations}'],
    'ekf_end': ['\\section{robot\\_localization Package'],
    'robot_loc_start': ['\\section{robot\\_localization Package'],
    'robot_loc_end': ['\\section{Complete Data Pipeline}', '% CHAPTER 9.6:'],
    'slam_theory_start': ['\\subsection{What is SLAM?}', '\\section{Theoretical Foundations'],
    'slam_theory_end': ['\\section{Hardware Specifications}'],
    'hardware_start': ['\\section{Hardware Specifications}'],
    'hardware_end': ['\\section{The v14\\_pro Configuration}'],
    'v14pro_start': ['\\section{The v14\\_pro Configuration'],
    'v14pro_end': ['\\section{Deployment Guide}'],
    'deployment_start': ['\\section{Deployment Guide}'],
    'deployment_end': ['\\section{Troubleshooting Guide}'],
    'troubleshooting_start': ['\\section{Troubleshooting Guide}'],
    'troubleshooting_end': ['\\section{Performance Benchmarks}'],
    'benchmarks_start': ['\\section{Performance Benchmarks}'],
    'benchmarks_end': ['\\section{Lessons Learned}'],
    'lessons_start': ['\\section{Lessons Learned}'],
    'lessons_end': ['\\section{Conclusion}', '\\section{Extended Kalman Filter}'],
    'conclusion_start': ['\\section{Conclusion}'],
    'pipeline_start': ['\\section{Complete Data Pipeline'],
    'pipeline_end': ['\\section{slam\\_toolbox Internals}'],
    'slam_toolbox_start': ['\\section{slam\\_toolbox Internals}'],
    'slam_toolbox_end': ['\\section{Hector SLAM vs slam\\_toolbox}'],
    'hector_vs_start': ['\\section{Hector SLAM vs slam\\_toolbox}'],
    'hector_vs_end': ['\\section{Launch File Analysis}'],
    'launch_file_start': ['\\section{Launch File Analysis}'],
    'launch_file_end': ['\\section{Final Summary}'],
    'final_summary_start': ['\\section{Final Summary}'],
}

# Find all sections
sections = find_section_boundaries(content, section_markers)

print("\nFound sections:")
for name, info in sorted(sections.items(), key=lambda x: x[1].get('start_pos', 999999)):
    if 'start_pos' in info:
        print(f"  {name:30s} at position {info['start_pos']:8d} (line {info['start_line']})")

# Extract key sections by finding content between markers
def extract_between(content, start_marker, end_marker, include_start=True):
    """Extract content between two string markers"""
    start_idx = content.find(start_marker)
    if start_idx == -1:
        return None

    if not include_start:
        start_idx += len(start_marker)

    end_idx = content.find(end_marker, start_idx + len(start_marker))
    if end_idx == -1:
        # If no end marker, take till end of file
        return content[start_idx:]

    return content[start_idx:end_idx]

# Extract preamble (document class, packages, title page, TOC)
preamble = content[:content.find('\\section{Executive Summary}')]

# Extract Executive Summary
exec_summary = extract_between(
    content,
    '\\section{Executive Summary}',
    '\\clearpage\n\n% ============================================================================\n% CHAPTER 1:'
)
if not exec_summary:
    exec_summary = extract_between(content, '\\section{Executive Summary}', '\\subsection{What is SLAM?}')

# Extract Arduino section (currently subsection under robot_localization)
arduino = extract_between(
    content,
    '% CHAPTER 9.6: ARDUINO MOTOR CONTROL',
    '\\subsection{Forward and Inverse Kinematics: The Mathematics'
)

# Extract Kinematics section
kinematics = extract_between(
    content,
    '\\subsection{Forward and Inverse Kinematics: The Mathematics',
    '\\clearpage\n\\section{Complete Data Pipeline'
)

# Extract EKF section
ekf = extract_between(
    content,
    '\\section{Extended Kalman Filter: Mathematical Foundations}',
    '\\section{robot\\_localization Package'
)

# Extract robot_localization section (but exclude Arduino and Kinematics subsections)
robot_loc = extract_between(
    content,
    '\\section{robot\\_localization Package',
    '% ============================================================================\n% CHAPTER 9.6: ARDUINO'
)

# Extract SLAM theory (currently mixed with Chapter 1)
slam_theory_start_idx = content.find('\\subsection{What is SLAM?}')
slam_theory_end_idx = content.find('\\section{Hardware Specifications}')
slam_theory = content[slam_theory_start_idx:slam_theory_end_idx]

# Extract remaining Part 2 sections
hardware = extract_between(content, '\\section{Hardware Specifications}', '\\section{The v14\\_pro Configuration}')
v14pro = extract_between(content, '\\section{The v14\\_pro Configuration}', '\\section{Deployment Guide}')
deployment = extract_between(content, '\\section{Deployment Guide}', '\\section{Troubleshooting Guide}')
troubleshooting = extract_between(content, '\\section{Troubleshooting Guide}', '\\section{Performance Benchmarks}')
benchmarks = extract_between(content, '\\section{Performance Benchmarks}', '\\section{Lessons Learned}')
lessons = extract_between(content, '\\section{Lessons Learned}', '\\section{Conclusion}')
conclusion_idx = content.find('\\section{Conclusion}')
ekf_idx = content.find('\\section{Extended Kalman Filter}')
if conclusion_idx != -1 and ekf_idx != -1 and conclusion_idx < ekf_idx:
    conclusion = content[conclusion_idx:ekf_idx]
else:
    conclusion = extract_between(content, '\\section{Conclusion}', '\\section{Extended Kalman Filter}')

pipeline = extract_between(content, '\\section{Complete Data Pipeline}', '\\section{slam\\_toolbox Internals}')
slam_toolbox = extract_between(content, '\\section{slam\\_toolbox Internals}', '\\section{Hector SLAM vs slam\\_toolbox}')
hector_vs = extract_between(content, '\\section{Hector SLAM vs slam\\_toolbox}', '\\section{Launch File Analysis}')
launch_file = extract_between(content, '\\section{Launch File Analysis}', '\\section{Final Summary}')
final_summary = extract_between(content, '\\section{Final Summary}', '\\end{document}')

print("\nExtracted sections (sizes):")
sections_extracted = {
    'preamble': preamble,
    'executive_summary': exec_summary,
    'arduino': arduino,
    'kinematics': kinematics,
    'ekf': ekf,
    'robot_loc': robot_loc,
    'slam_theory': slam_theory,
    'hardware': hardware,
    'v14pro': v14pro,
    'deployment': deployment,
    'troubleshooting': troubleshooting,
    'benchmarks': benchmarks,
    'lessons': lessons,
    'conclusion': conclusion,
    'pipeline': pipeline,
    'slam_toolbox': slam_toolbox,
    'hector_vs': hector_vs,
    'launch_file': launch_file,
    'final_summary': final_summary,
}

for name, section in sections_extracted.items():
    if section:
        print(f"  {name:20s}: {len(section):8d} chars")
    else:
        print(f"  {name:20s}: NOT FOUND!")

# Build PART dividers
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

part2_divider = """\\clearpage

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

# Promote Arduino and Kinematics from subsections to sections
if arduino:
    arduino = arduino.replace('\\subsection{Arduino Motor Control: The Low-Level Implementation}',
                             '\\section{Arduino Motor Control: The Low-Level Implementation}')
    # Promote all subsubsections to subsections
    arduino = arduino.replace('\\subsubsection{', '\\subsection{')
    # Fix the comment
    arduino = arduino.replace('% CHAPTER 9.6: ARDUINO MOTOR CONTROL',
                             '% CHAPTER 1: ARDUINO MOTOR CONTROL')

if kinematics:
    kinematics = kinematics.replace('\\subsection{Forward and Inverse Kinematics: The Mathematics of Differential Drive}',
                                   '\\section{Forward and Inverse Kinematics: The Mathematics of Differential Drive}')
    # Promote all subsubsections to subsections
    kinematics = kinematics.replace('\\subsubsection{', '\\subsection{')
    # Add chapter comment
    kinematics = '\n% ============================================================================\n% CHAPTER 2: FORWARD AND INVERSE KINEMATICS\n% ============================================================================\n' + kinematics

# Fix SLAM theory to be a proper section
if slam_theory:
    slam_theory = '% ============================================================================\n% CHAPTER 6: SLAM THEORETICAL FOUNDATIONS\n% ============================================================================\n\\section{Theoretical Foundations of 2D LiDAR SLAM}\n\n' + slam_theory
    # Promote subsections
    slam_theory = slam_theory.replace('\\subsection{What is SLAM?}', '\\subsection{What is SLAM?}')  # Keep as-is

# Reassemble in new order
new_content = preamble

# Add Executive Summary
if exec_summary:
    new_content += exec_summary

# PART 1
new_content += part1_divider

# Chapter 1: Arduino
if arduino:
    new_content += arduino
else:
    print("WARNING: Arduino section not found!")

# Chapter 2: Kinematics
if kinematics:
    new_content += kinematics
else:
    print("WARNING: Kinematics section not found!")

# Chapter 3: EKF
if ekf:
    ekf = ekf.replace('% CHAPTER 8:', '% CHAPTER 3:')
    new_content += ekf
else:
    print("WARNING: EKF section not found!")

# Chapter 4: robot_localization
if robot_loc:
    robot_loc = robot_loc.replace('% CHAPTER 9:', '% CHAPTER 4:')
    new_content += robot_loc
else:
    print("WARNING: robot_localization section not found!")

# PART 2
new_content += part2_divider

# Chapter 5: Hardware (was Chapter 2)
if hardware:
    hardware = hardware.replace('% CHAPTER 2:', '% CHAPTER 5:')
    new_content += hardware

# Chapter 6: SLAM Theory (was Chapter 1)
if slam_theory:
    new_content += slam_theory

# Chapter 7: v14_pro (was Chapter 3)
if v14pro:
    v14pro = v14pro.replace('% CHAPTER 3:', '% CHAPTER 7:')
    new_content += v14pro

# Chapter 8: Data Pipeline (was Chapter 10)
if pipeline:
    pipeline = pipeline.replace('% CHAPTER 10:', '% CHAPTER 8:')
    new_content += pipeline

# Chapter 9: slam_toolbox (was Chapter 11)
if slam_toolbox:
    slam_toolbox = slam_toolbox.replace('% CHAPTER 11:', '% CHAPTER 9:')
    new_content += slam_toolbox

# Chapter 10: Hector vs slam_toolbox (was Chapter 12)
if hector_vs:
    hector_vs = hector_vs.replace('% CHAPTER 12:', '% CHAPTER 10:')
    new_content += hector_vs

# Chapter 11: Launch Files (was Chapter 13)
if launch_file:
    launch_file = launch_file.replace('% CHAPTER 13:', '% CHAPTER 11:')
    new_content += launch_file

# Chapter 12: Deployment (was Chapter 4)
if deployment:
    deployment = deployment.replace('% CHAPTER 4:', '% CHAPTER 12:')
    new_content += deployment

# Chapter 13: Troubleshooting (was Chapter 5)
if troubleshooting:
    troubleshooting = troubleshooting.replace('% CHAPTER 5:', '% CHAPTER 13:')
    new_content += troubleshooting

# Chapter 14: Benchmarks (was Chapter 6)
if benchmarks:
    benchmarks = benchmarks.replace('% CHAPTER 6:', '% CHAPTER 14:')
    new_content += benchmarks

# Chapter 15: Lessons Learned (was Chapter 7)
if lessons:
    lessons = lessons.replace('% CHAPTER 7:', '% CHAPTER 15:')
    new_content += lessons

# Chapter 16: Final Summary (was Chapter 14)
if final_summary:
    final_summary = final_summary.replace('% CHAPTER 14:', '% CHAPTER 16:')
    new_content += final_summary

# Add end of document
new_content += '\n\\end{document}\n'

print(f"\nNew file size: {len(new_content)} characters")
print(f"Size difference: {len(new_content) - len(content)} characters")

# Write to new file
write_file('2D_LIDAR_SLAM_COMPLETE_GUIDE.tex', new_content)

print("\n" + "=" * 70)
print("REORGANIZATION COMPLETE!")
print("=" * 70)
print(f"\nOriginal backed up to: 2D_LIDAR_SLAM_COMPLETE_GUIDE_BACKUP.tex")
print(f"New structure written to: 2D_LIDAR_SLAM_COMPLETE_GUIDE.tex")
print("\nNew chapter order:")
print("  PART 1: Low-Level Control and Odometry")
print("    Ch 1: Arduino Motor Control")
print("    Ch 2: Forward and Inverse Kinematics")
print("    Ch 3: Extended Kalman Filter")
print("    Ch 4: robot_localization Package")
print("  PART 2: Mapping, Localization and SLAM")
print("    Ch 5: Hardware Specifications")
print("    Ch 6: SLAM Theoretical Foundations")
print("    Ch 7: v14_pro Configuration")
print("    Ch 8: Complete Data Pipeline")
print("    Ch 9: slam_toolbox Internals")
print("    Ch 10: Hector SLAM vs slam_toolbox")
print("    Ch 11: Launch File Analysis")
print("    Ch 12: Deployment Guide")
print("    Ch 13: Troubleshooting")
print("    Ch 14: Performance Benchmarks")
print("    Ch 15: Lessons Learned")
print("    Ch 16: Final Summary")
