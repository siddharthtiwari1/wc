# CLAUDE.md - AI Assistant Development Guide

**Last Updated**: 2025-11-20
**Project**: wc
**Repository**: siddharthtiwari1/wc
**Owner**: Siddharth Tiwari (s24035@students.iitmandi.ac.in)

---

## Project Overview

### What is this project?

This is the **wc** project - currently in the initial development stage. Based on the project name, this appears to be a word count utility implementation (similar to the Unix `wc` command).

### Current Status

**Stage**: Early initialization
**Branch**: `claude/claude-md-mi6y8g2to5fjphmj-01N6TcZncM17wVYW1FzAFmyc`
**Latest Commit**: Add initial README with project title (8bd6c23)

The repository currently contains:
- Basic README.md with project title
- Git repository structure
- This CLAUDE.md file (AI assistant guide)

---

## Repository Structure

```
/home/user/wc/
├── .git/                    # Git repository metadata
├── README.md                # Project documentation (minimal)
├── CLAUDE.md               # This file - AI assistant guide
└── (additional files to be added)
```

### Expected Future Structure

As this project develops, expect to see:

```
/home/user/wc/
├── src/                    # Source code directory
│   └── (implementation files)
├── tests/                  # Test files
│   └── (test files)
├── docs/                   # Additional documentation
├── .gitignore             # Git ignore rules
├── README.md              # User-facing documentation
├── CLAUDE.md              # This file
├── LICENSE                # Project license
└── (build configuration)  # Language-specific config files
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

### What "wc" Typically Means
The Unix `wc` (word count) command counts:
- Lines (`-l` flag)
- Words (`-w` flag)
- Characters (`-c` flag)
- Bytes (`-c` flag)
- Max line length (`-L` flag)

If this project is implementing a similar utility, consider:
- Multiple input sources (stdin, files)
- Multiple counting modes
- Efficient processing of large files
- POSIX compatibility (if applicable)
- Edge cases (empty files, binary files, etc.)

### Design Considerations
- **Performance**: Should handle large files efficiently
- **Correctness**: Define what counts as a "word" or "line"
- **Compatibility**: Match Unix wc behavior or define differences
- **Error Handling**: Graceful handling of missing/unreadable files
- **Testing**: Comprehensive test cases for edge cases

---

## Environment Information

**Platform**: Linux
**OS Version**: Linux 4.4.0
**Git Repository**: Yes
**Working Directory**: `/home/user/wc`

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
- [README.md](README.md) - User-facing documentation

### External Resources
- Unix wc manual: `man wc`
- POSIX wc specification: [POSIX.1-2017](https://pubs.opengroup.org/onlinepubs/9699919799/utilities/wc.html)

---

## Notes for Future Development

This section should be updated as the project evolves:

- **Architecture Decisions**: Document major design choices
- **Known Issues**: Track known bugs or limitations
- **Performance Benchmarks**: Record performance metrics
- **Breaking Changes**: Track API/behavior changes
- **Migration Guides**: Help for updating to new versions

---

*This file is maintained for AI assistants (like Claude) working on this codebase. It provides context, conventions, and guidelines for effective collaboration.*
