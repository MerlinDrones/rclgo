## Contributing to rclgo

Thank you for your interest in contributing to rclgo! This document provides guidelines and best practices for contributing to the project.

## Table of Contents

- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Branching Strategy](#branching-strategy)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Testing](#testing)
- [Code Style](#code-style)
- [Multi-Distro Development](#multi-distro-development)

## Getting Started

### Prerequisites

- Go 1.24 or later
- ROS 2 (Humble or Jazzy) installed
- Docker (for cross-distro testing)
- `golangci-lint` for linting
- `pre-commit` framework (optional but recommended)

### Initial Setup

1. **Fork and clone the repository**:
   ```bash
   git fork https://github.com/MerlinDrones/rclgo
   git clone git@github.com:YOUR_USERNAME/rclgo.git
   cd rclgo
   ```

2. **Add upstream remote**:
   ```bash
   git remote add upstream git@github.com:MerlinDrones/rclgo.git
   ```

3. **Install pre-commit hooks** (recommended):
   ```bash
   pip install pre-commit
   pre-commit install
   ```

4. **Source ROS environment**:
   ```bash
   source /opt/ros/humble/setup.bash  # or jazzy
   ```

5. **Verify build**:
   ```bash
   make build
   go build ./...
   go test ./...
   ```

## Development Workflow

### Quick Start: Making a Change

1. **Create a feature branch from the target distro**:
   ```bash
   # Using the helper script (recommended)
   ./scripts/git-rclgo.sh branch-from humble

   # Or manually
   git checkout humble
   git pull origin humble
   git checkout -b feature/my-awesome-feature
   ```

2. **Make your changes** and commit frequently:
   ```bash
   # Make changes...
   git add <files>
   git commit -m "feat(scope): add awesome feature"
   ```

3. **Test your changes**:
   ```bash
   go build ./...
   go test ./...
   make lint  # If golangci-lint is installed
   ```

4. **Push and create a PR**:
   ```bash
   git push -u origin feature/my-awesome-feature
   gh pr create --base humble --fill
   ```

5. **Address review feedback** and update your PR:
   ```bash
   # Make changes...
   git add <files>
   git commit -m "fix: address review comments"
   git push
   ```

### Daily Workflow

**Start of day**:
```bash
git checkout humble
git pull origin humble
git checkout feature/my-feature
git rebase humble  # Keep your branch up to date
```

**During work** (commit every 15-30 minutes):
```bash
git add -p  # Stage changes interactively
git commit  # Write meaningful commit message
```

**End of day** (or when ready to share):
```bash
git push -u origin feature/my-feature
```

## Branching Strategy

rclgo uses a **multi-distro branching model** where each ROS 2 distro has its own long-lived branch.

### Branch Types

| Branch Type | Naming | Purpose | Base Branch |
|------------|--------|---------|-------------|
| **Distro branches** | `humble`, `jazzy` | Long-lived distro-specific branches | N/A |
| **Feature branches** | `feature/<name>` | New features | Target distro (usually `humble`) |
| **Fix branches** | `fix/<name>` | Bug fixes | Affected distro |
| **Sync branches** | `sync/<from>-to-<to>` | Cross-distro porting | Target distro |
| **Experiment branches** | `experiment/<name>` | Experimental work | Any distro |

### Protected Branches

The following branches are protected and require PRs:
- `humble` (primary distro)
- `jazzy`
- `master` (points to primary distro)

**You cannot commit directly to these branches.** The pre-commit hooks will prevent this.

## Commit Guidelines

### Commit Message Format

We follow [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <subject>

[optional body]

[optional footer]
```

**Types**:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `chore`: Maintenance tasks (deps, tooling)
- `test`: Adding or updating tests
- `refactor`: Code refactoring
- `build`: Build system changes
- `ci`: CI/CD changes
- `perf`: Performance improvements
- `sync`: Cross-distro sync

**Scopes** (optional):
- Package/component name: `params`, `qos`, `rostime`, `logging`, `action`
- Distro-specific: `humble`, `jazzy`
- Tool: `rclgo-gen`, `gogen`

**Examples**:
```bash
feat(params): add YAML parameter loading
fix(jazzy): update CGO bindings for Jazzy API change
sync(jazzy): port logging feature from humble
docs(ROADMAP): update QoS implementation status
chore(deps): update golang.org/x/tools to v0.38.0
```

### Commit Frequency

- **Commit often**: Every 15-30 minutes or when a logical unit is complete
- **Atomic commits**: One logical change per commit
- **Build should pass**: Every commit should compile

### What Makes a Good Commit?

✅ **GOOD**:
```
feat(logging): add severity level filtering

Implements LogSeverity enum and filtering in Logger.GetEffectiveSeverity().
Adds tests for all severity levels.

Closes #123
```

❌ **BAD**:
```
work on stuff

- changed logging
- fixed some params
- updated readme
```

## Pull Request Process

### Before Creating a PR

1. ✅ Your branch builds successfully
2. ✅ All tests pass
3. ✅ Linting passes (if `golangci-lint` is installed)
4. ✅ You've tested on the target ROS 2 distro
5. ✅ Commit messages follow conventional format

### Creating a PR

Use the PR template (auto-populated) and fill out all sections:

```bash
gh pr create --base humble --fill
```

Or create via GitHub UI - the template will guide you.

### PR Review Checklist

When reviewing your own PR (even for solo work):
- [ ] All CI checks pass (tests, linting, build)
- [ ] Code is documented (complex logic has comments)
- [ ] New features have tests
- [ ] No debugging statements (`fmt.Println`, `TODO`, etc.)
- [ ] Generated code is up to date (`make generate` if needed)
- [ ] ROADMAP.md updated if this affects feature parity
- [ ] Consider cross-distro impact (needs syncing?)

### Merging

- **Squash and merge**: For feature branches with many small commits
- **Merge commit**: For sync branches (preserves history)
- **Rebase and merge**: For clean, atomic commits

After merge:
- Delete your feature branch (GitHub will prompt)
- Consider syncing to other distros if applicable

## Testing

### Local Testing

```bash
# Build
go build ./...

# Run tests
go test ./...

# Run tests with coverage
go test -v -race -coverprofile=coverage.txt -covermode=atomic ./...

# Run fast tests only (for pre-commit)
go test -short ./...

# Test specific package
go test ./pkg/rclgo/params/...
```

### Cross-Distro Testing

Use Docker to test on multiple distros:

```bash
# Test on all distros
./scripts/test-all-distros.sh

# Test on specific distro
./scripts/test-all-distros.sh --distro=jazzy

# Verbose output
./scripts/test-all-distros.sh --verbose
```

### Manual Testing with ROS 2

```bash
# Build and test publisher example
source /opt/ros/humble/setup.bash
cd examples/publisher_subscriber
go build ./...
./publisher/publisher
```

## Code Style

### Go Style Guide

- Follow [Effective Go](https://go.dev/doc/effective_go)
- Use `go fmt` (enforced by pre-commit hooks)
- Run `go vet` (enforced by pre-commit hooks)
- Use `golangci-lint` (enforced in CI)

### rclgo-Specific Conventions

**Naming**:
- Use ROS 2 terminology where applicable
- Exported types/functions use PascalCase
- Internal helpers use camelCase
- CGO functions prefix with `C.`

**Error Handling**:
```go
// Wrap RCL errors with context
if rc != C.RCL_RET_OK {
    return fmt.Errorf("rcl_init failed: %w", errorsCast(rc))
}
```

**Testing**:
- Test files: `*_test.go`
- Table-driven tests where applicable
- Use `testify/assert` for assertions

**Documentation**:
- All exported types/functions must have godoc comments
- Complex algorithms need inline comments
- CGO code should explain memory management

### Generated Code

**DO NOT** manually edit generated files:
- `pkg/msgs/**/*.gen.go`
- `*.pb.go`

Instead, regenerate:
```bash
make generate
```

## Multi-Distro Development

### Cross-Distro Features

Most features apply to all distros. Workflow:

1. **Develop on primary distro** (humble):
   ```bash
   git checkout -b feature/awesome-thing humble
   # ... develop, test, commit ...
   ```

2. **Create PR to humble**:
   ```bash
   gh pr create --base humble --fill
   ```

3. **After merge, sync to jazzy**:
   ```bash
   ./scripts/sync-feature.sh humble jazzy <commit-range>
   # ... test on jazzy ...
   gh pr create --base jazzy --title "sync: awesome-thing from humble" --fill
   ```

### Distro-Specific Changes

Some changes only apply to one distro (CGO bindings, API differences):

1. **Branch from target distro**:
   ```bash
   git checkout -b fix/jazzy-cgo-issue jazzy
   ```

2. **Make distro-specific changes**

3. **Create PR with `distro-specific` label**:
   ```bash
   gh pr create --base jazzy --label "distro-specific" --fill
   ```

### Testing Across Distros

Before syncing, verify your change works on both distros:

```bash
# Test on humble
source /opt/ros/humble/setup.bash
go test ./...

# Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# Or use Docker
./scripts/test-all-distros.sh
```

## Helper Tools

### git-rclgo

Multi-distro workflow helper:

```bash
# Create feature branch
./scripts/git-rclgo.sh branch-from humble

# Show current distro context
./scripts/git-rclgo.sh which-distro

# Sync between distros
./scripts/git-rclgo.sh sync humble jazzy <commits>

# Quick WIP commit
./scripts/git-rclgo.sh quick-save "debugging logging"

# Undo last commit (keep changes)
./scripts/git-rclgo.sh oops
```

### Useful Git Aliases

Add to your `~/.gitconfig`:

```ini
[alias]
    # rclgo-specific
    rclgo = !bash /path/to/rclgo/scripts/git-rclgo.sh

    # General helpers
    st = status -sb
    co = checkout
    br = branch
    ci = commit
    unstage = reset HEAD --
    last = log -1 HEAD
    visual = log --oneline --graph --decorate --all
```

## Getting Help

- **Documentation**: See `docs/BRANCHING.md` and `docs/RELEASING.md`
- **Issues**: https://github.com/MerlinDrones/rclgo/issues
- **Discussions**: Open an issue for questions
- **ROADMAP**: See `ROADMAP.md` for feature status

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

---

Thank you for contributing to rclgo! 🚀
