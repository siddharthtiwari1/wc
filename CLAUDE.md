# CLAUDE.md - AI Assistant Development Guide

**Last Updated**: 2025-11-25
**Project**: Wheelchair Autonomous Navigation System
**Repository**: siddharthtiwari1/wc
**Owner**: Siddharth Tiwari (s24035@students.iitmandi.ac.in)

---

## REFERENCE LEARNING DOCUMENTS

**IMPORTANT**: Before working on navigation, localization, or Nav2 related tasks, review these learning documents:

| Document | Path | Description |
|----------|------|-------------|
| **Bumperbot Nav2 Learnings** | [`docs/BUMPERBOT_LEARNINGS.md`](docs/BUMPERBOT_LEARNINGS.md) | Comprehensive Nav2 patterns from bumperbot_ws |

### Source Workspaces for Learning
- **Bumperbot WS**: `/home/sidd/Downloads/lidartest-20251121T154203Z-1-001/lidartest/src/bumperbot_ws`
  - Well-structured Nav2 navigation with custom planners (A*, Dijkstra)
  - Custom controllers (Pure Pursuit, PD Motion)
  - Safety features, behavior trees, EKF/AMCL localization

---

## CRITICAL ROS2 RULES - ALWAYS FOLLOW

### Before ANY ROS2 Launch
**ALWAYS** kill all existing ROS2 processes and verify they are dead before launching:

```bash
# Step 1: Kill all ROS2 processes
pkill -9 -f ros2; pkill -9 -f rviz; pkill -9 -f gzclient; pkill -9 -f gzserver

# Step 2: Wait for cleanup
sleep 3

# Step 3: Verify no processes running
pgrep -a ros2 || echo "Clean - ready to launch"

# Step 4: Check ROS2 topics are cleared
ros2 topic list 2>/dev/null | wc -l  # Should be 0 or very few

# Step 5: Only then launch
ros2 launch wheelchair_bringup wheelchair_global_localization.launch.py
```

**WHY**: USB devices (RealSense, RPLidar, Arduino) get locked by previous processes. Not killing them causes "Device busy" errors.

### Main Launch Files
- **Localization**: `ros2 launch wheelchair_bringup wheelchair_global_localization.launch.py`
- **Autonomous Nav**: `ros2 launch wheelchair_bringup wheelchair_autonomous_nav.launch.py`
- **Default Map**: `/home/sidd/wc/maps/my_map.yaml`

---

## Project Overview

### What is this project?

This is the **Wheelchair Autonomous Navigation System** - a ROS2 Jazzy-based autonomous wheelchair with SLAM, localization, and Nav2 navigation.

### Current Status

**Stage**: Autonomous Navigation POC
**Branch**: `main`
**Working Directory**: `/home/sidd/wc`

The system currently has:
- Working odometry and EKF localization
- AMCL global localization (needs fine-tuning)
- Pre-built map at `/home/sidd/wc/maps/my_map.yaml`
- Full Nav2 navigation stack configured

### Hardware Components
- **Wheelchair Base**: Custom differential drive with Arduino interface
- **LIDAR**: RPLidar S3 (360 deg scan on /scan)
- **Camera**: RealSense D455 (depth + RGB)
- **IMU**: Internal sensor for orientation

---

## Repository Structure

```
/home/sidd/wc/
├── src/
│   ├── wheelchair_bringup/          # Main launch files
│   ├── wheelchair_navigation/       # Nav2 configs, behavior trees
│   ├── wheelchair_localization/     # EKF, AMCL configs
│   ├── wheelchair_planning/         # Custom planners (A*, Dijkstra)
│   ├── wheelchair_motion/           # Custom controllers (Pure Pursuit, PD)
│   ├── wheelchair_description/      # URDF, RViz configs
│   ├── wheelchair_firmware/         # Arduino hardware interface
│   └── wc_control/                  # ros2_control configs
├── maps/                            # Saved maps
│   └── my_map.yaml                  # Current production map
├── docs/                            # Documentation
│   └── BUMPERBOT_LEARNINGS.md       # Nav2 patterns from bumperbot
├── build/                           # Colcon build output
├── install/                         # Colcon install output
├── CLAUDE.md                        # This file
└── README.md                        # Project documentation
```

---

## Development Workflows

### Git Branch Strategy

**Current Development Branch**: `claude/claude-md-mi6y8g2to5fjphmj-01N6TcZncM17wVYW1FzAFmyc`

#### Branch Naming Convention
- Claude-assisted development branches: `claude/claude-md-{session-id}`
- Feature branches: (to be defined)
- Main branch: (to be determined)

#### Git Operations Best Practices

**Pushing Changes:**
```bash
# Always use -u flag for first push
git push -u origin claude/claude-md-mi6y8g2to5fjphmj-01N6TcZncM17wVYW1FzAFmyc

# Retry logic for network failures (up to 4 times with exponential backoff)
# Delays: 2s, 4s, 8s, 16s
```

**Fetching/Pulling:**
```bash
# Prefer specific branch fetches
git fetch origin <branch-name>
git pull origin <branch-name>
```

**Committing:**
- Write clear, descriptive commit messages
- Follow conventional commits format (recommended):
  - `feat:` for new features
  - `fix:` for bug fixes
  - `docs:` for documentation
  - `refactor:` for code refactoring
  - `test:` for test additions/changes
  - `chore:` for maintenance tasks

### Development Process

1. **Understand the Request**: Analyze what needs to be done
2. **Plan with TodoWrite**: Use the TodoWrite tool to break down tasks
3. **Implement Changes**: Make code changes incrementally
4. **Test Changes**: Verify functionality works as expected
5. **Commit Work**: Commit with clear messages
6. **Push to Branch**: Push to the designated Claude branch

---

## Key Conventions for AI Assistants

### File Operations

**DO:**
- Use `Read` for reading files (not `cat`)
- Use `Edit` for modifying existing files (not `sed`/`awk`)
- Use `Write` for creating new files (not `echo` or heredoc)
- Use `Glob` for finding files by pattern
- Use `Grep` for searching file contents

**DON'T:**
- Don't use bash commands for file operations when specialized tools exist
- Don't create unnecessary files (especially markdown/docs unless requested)
- Don't use emojis unless explicitly requested
- Don't use `echo` to communicate with users (output text directly)

### Code Quality Standards

#### Security
- **Always** check for security vulnerabilities:
  - Command injection
  - XSS (Cross-Site Scripting)
  - SQL injection
  - OWASP Top 10 vulnerabilities
- Fix security issues immediately upon detection

#### Best Practices
- Write clear, readable code with appropriate comments
- Follow language-specific style guides
- Prefer existing patterns in the codebase
- Keep functions small and focused
- Write tests for new functionality

### Communication Style

- **Concise**: Keep responses short and to the point
- **Technical**: Focus on accuracy and facts
- **Objective**: Prioritize truth over validation
- **Direct**: Use GitHub-flavored markdown, rendered in monospace
- **No Emojis**: Unless explicitly requested by the user

### Task Management

**CRITICAL**: Always use `TodoWrite` tool for:
- Planning multi-step tasks (3+ steps)
- Complex non-trivial tasks
- When user provides multiple tasks
- Tracking implementation progress

**Task States:**
- `pending`: Not started
- `in_progress`: Currently working (ONLY ONE at a time)
- `completed`: Finished successfully

**Task Completion Rules:**
- Mark completed IMMEDIATELY after finishing
- Don't batch completions
- Only mark completed when FULLY done (not if tests fail or errors occur)
- Keep tasks specific and actionable

---

## Language and Framework Guidelines

### To Be Determined

As the project develops, add sections for:
- **Programming Language**: (Python, JavaScript, Go, Rust, C, etc.)
- **Framework/Libraries**: Dependencies and versions
- **Build System**: (npm, pip, cargo, make, etc.)
- **Testing Framework**: (pytest, jest, go test, etc.)
- **Code Style**: Linter and formatter configurations
- **Package Management**: How dependencies are managed

---

## Testing

### Current State
No testing framework configured yet.

### Future Testing Strategy
Document here:
- Test framework choice
- Test directory structure
- How to run tests
- Coverage requirements
- CI/CD integration

---

## Build and Deployment

### Current State
No build system configured yet.

### Future Build Process
Document here:
- Build commands
- Build artifacts
- Deployment process
- Environment configuration
- CI/CD pipelines

---

## Common Tasks Reference

### For AI Assistants Working on This Project

#### Adding New Features
1. Use TodoWrite to plan the feature implementation
2. Create/modify source files in appropriate directories
3. Add tests for new functionality
4. Update documentation (README.md)
5. Commit with descriptive message
6. Push to the Claude development branch

#### Fixing Bugs
1. Identify the bug location and cause
2. Write a failing test (if possible)
3. Fix the bug
4. Verify the test passes
5. Commit with "fix:" prefix
6. Push changes

#### Updating Documentation
1. Read existing documentation first
2. Make clear, concise updates
3. Ensure accuracy
4. Commit with "docs:" prefix
5. Push changes

#### Refactoring Code
1. Ensure tests exist for the code being refactored
2. Make incremental changes
3. Run tests after each change
4. Commit with "refactor:" prefix
5. Push changes

---

## Project-Specific Context

### What "wc" Means Here
**WC = WheelChair** - This is an autonomous wheelchair navigation project, NOT the Unix word count utility.

### System Architecture

```
                    ┌─────────────────────────────────────────────────────┐
                    │            WHEELCHAIR AUTONOMOUS NAV                │
                    └─────────────────────────────────────────────────────┘
                                           │
           ┌───────────────────────────────┼───────────────────────────────┐
           │                               │                               │
    ┌──────▼──────┐               ┌────────▼────────┐             ┌────────▼────────┐
    │   SENSORS   │               │   LOCALIZATION  │             │   NAVIGATION    │
    └──────┬──────┘               └────────┬────────┘             └────────┬────────┘
           │                               │                               │
    ┌──────┴──────┐               ┌────────┴────────┐             ┌────────┴────────┐
    │ RPLidar S3  │               │ EKF (odom+IMU)  │             │ Nav2 Stack      │
    │ /scan       │               │ /odometry/filt. │             │ - Controller    │
    ├─────────────┤               ├─────────────────┤             │ - Planner       │
    │ RealSense   │               │ AMCL (map loc)  │             │ - BT Navigator  │
    │ /camera/*   │               │ map→odom TF     │             │ - Behaviors     │
    ├─────────────┤               └─────────────────┘             └─────────────────┘
    │ Wheel Odom  │                       │                               │
    │ /wc_control │                       │                               │
    └─────────────┘                       └───────────────┬───────────────┘
                                                          │
                                                   ┌──────▼──────┐
                                                   │  /cmd_vel   │
                                                   └──────┬──────┘
                                                          │
                                                   ┌──────▼──────┐
                                                   │   Arduino   │
                                                   │ Motor Ctrl  │
                                                   └─────────────┘
```

### Key Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | RPLidar S3 360° scan |
| `/odometry/filtered` | Odometry | EKF fused odometry |
| `/map` | OccupancyGrid | Static map from map_server |
| `/cmd_vel` | Twist | Navigation velocity commands |
| `/wc_control/cmd_vel` | Twist | Arduino motor interface |

### TF Tree
```
map → odom → base_link → base_laser → rplidar_link
                │
                └→ camera_link → camera_depth_frame
```

---

## Environment Information

**Platform**: Linux
**OS Version**: Linux 6.14.0 (Ubuntu)
**ROS2 Version**: Jazzy
**Git Repository**: Yes
**Working Directory**: `/home/sidd/wc`

---

## Tool Usage Guidelines

### Exploration and Search
- Use `Task` tool with `Explore` subagent for codebase exploration
- Use `Grep` for content searches (not grep via Bash)
- Use `Glob` for file pattern matching

### Parallel Operations
- Make independent tool calls in parallel when possible
- Use sequential calls only when there are dependencies
- Never use placeholders for missing parameters

### Git Operations
- **No `gh` CLI**: GitHub CLI is not available; request info from user
- **Push requires correct branch**: Must match `claude/` prefix and session ID
- **Network retry**: Up to 4 retries with exponential backoff

---

## Questions to Clarify with User

When developing this project, consider asking:

1. **Language Choice**: What programming language should be used?
2. **Feature Scope**: Should this match Unix `wc` exactly, or have different features?
3. **Platform Target**: Is this for Linux only, or cross-platform?
4. **Dependencies**: Are external libraries allowed, or should it be standalone?
5. **Performance Goals**: What file sizes should be handled efficiently?
6. **Testing Requirements**: What level of test coverage is expected?

---

## Maintenance Notes

### Updating This File

This CLAUDE.md should be updated when:
- Project structure changes significantly
- New conventions are established
- Build/test processes are defined
- New frameworks or languages are added
- Important architectural decisions are made

**Last Structural Change**: Initial creation (2025-11-20)

---

## Additional Resources

### Internal Documentation
- [README.md](README.md) - Project overview
- [docs/BUMPERBOT_LEARNINGS.md](docs/BUMPERBOT_LEARNINGS.md) - Nav2 patterns and configurations

### External Resources
- Nav2 Documentation: https://navigation.ros.org/
- ROS2 Jazzy Docs: https://docs.ros.org/en/jazzy/
- robot_localization Wiki: http://docs.ros.org/en/melodic/api/robot_localization/html/

---

## Quick Reference Commands

### Launch System
```bash
# Kill all ROS2 processes first
pkill -9 -f ros2; pkill -9 -f rviz; sleep 3

# Localization only
ros2 launch wheelchair_bringup wheelchair_global_localization.launch.py

# Full autonomous navigation
ros2 launch wheelchair_bringup wheelchair_autonomous_nav.launch.py
```

### Debugging
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check Nav2 lifecycle states
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server

# Check costmaps updating
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap

# Send test navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

### Building
```bash
cd /home/sidd/wc
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Notes for Future Development

### Current Priorities
1. Fine-tune AMCL localization accuracy
2. Test autonomous navigation POC end-to-end
3. Add safety features (collision monitor, safety stop)

### Known Issues
- AMCL localization needs tuning for better accuracy
- Depth camera not yet integrated into costmaps

### Architecture Decisions
- Using Regulated Pure Pursuit controller for smooth wheelchair motion
- SMAC Planner 2D for efficient path planning
- Velocity smoother for passenger comfort

---

*This file is maintained for AI assistants (like Claude) working on this codebase. It provides context, conventions, and guidelines for effective collaboration.*
