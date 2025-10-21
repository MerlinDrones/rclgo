# rclgo Helper Scripts

This directory contains helper scripts for working with rclgo's multi-distro git workflow.

## Scripts

### `git-rclgo.sh`

Multi-purpose helper for rclgo development workflows.

**Usage**:
```bash
./scripts/git-rclgo.sh <command> [args...]
```

**Commands**:

| Command | Description | Example |
|---------|-------------|---------|
| `branch-from <distro>` | Create feature branch from distro | `./scripts/git-rclgo.sh branch-from humble` |
| `sync <from> <to> [commits]` | Port commits between distros | `./scripts/git-rclgo.sh sync humble jazzy HEAD~3..HEAD` |
| `which-distro` | Show current distro context | `./scripts/git-rclgo.sh which-distro` |
| `status` | Show git status with distro info | `./scripts/git-rclgo.sh status` |
| `test-all` | Run tests on all distros (Docker) | `./scripts/git-rclgo.sh test-all` |
| `safe-push` | Push with full checks | `./scripts/git-rclgo.sh safe-push` |
| `quick-save [msg]` | Quick WIP commit | `./scripts/git-rclgo.sh quick-save "debugging"` |
| `oops` | Undo last commit (keep changes) | `./scripts/git-rclgo.sh oops` |

**Examples**:
```bash
# Start a new feature
./scripts/git-rclgo.sh branch-from humble
# Follow interactive prompts...

# Check which distro you're working on
./scripts/git-rclgo.sh which-distro

# Quick save your work
./scripts/git-rclgo.sh quick-save "WIP: implementing param validation"

# Undo last commit (if you made a mistake)
./scripts/git-rclgo.sh oops
```

**Install as git alias**:
```bash
git config --global alias.rclgo '!bash /path/to/rclgo/scripts/git-rclgo.sh'

# Then use as:
git rclgo branch-from humble
git rclgo which-distro
```

---

### `sync-feature.sh`

Semi-automated script for porting features between distros.

**Usage**:
```bash
./scripts/sync-feature.sh <from-distro> <to-distro> [commit-range]
```

**Arguments**:
- `from-distro`: Source distro (humble, jazzy)
- `to-distro`: Target distro (humble, jazzy)
- `commit-range`: Git commit range to cherry-pick (optional, will prompt if not provided)

**Examples**:
```bash
# Port recent commits from humble to jazzy
./scripts/sync-feature.sh humble jazzy HEAD~5..HEAD

# Port specific commit range
./scripts/sync-feature.sh humble jazzy abc123..def456

# Interactive mode (will show commits and prompt)
./scripts/sync-feature.sh humble jazzy
```

**What it does**:
1. Fetches latest from both distros
2. Shows commits in the specified range
3. Prompts for confirmation
4. Creates a `sync/<from>-to-<to>` branch
5. Cherry-picks commits one by one
6. Offers to run tests
7. Provides next steps (push, create PR)

**Handling conflicts**:
If cherry-pick fails with conflicts:
```bash
# 1. Fix conflicts in your editor
# 2. Stage resolved files
git add <fixed-files>

# 3. Continue cherry-pick
git cherry-pick --continue

# Or skip the commit
git cherry-pick --skip

# Or abort entirely
git cherry-pick --abort
```

---

### `test-all-distros.sh`

Test rclgo on all supported ROS 2 distros using Docker.

**Usage**:
```bash
./scripts/test-all-distros.sh [options]
```

**Options**:
- `--verbose, -v`: Show verbose output
- `--distro=<distro>`: Test only specific distro (humble, jazzy)
- `--help, -h`: Show help message

**Examples**:
```bash
# Test on all distros
./scripts/test-all-distros.sh

# Test with verbose output
./scripts/test-all-distros.sh --verbose

# Test only on Humble
./scripts/test-all-distros.sh --distro=humble

# Test only on Jazzy
./scripts/test-all-distros.sh --distro=jazzy
```

**Requirements**:
- Docker installed and running
- Internet connection (to pull ROS images)

**What it does**:
1. Pulls official ROS 2 Docker images
2. Mounts rclgo repo into containers
3. Installs dependencies
4. Runs full build and test suite
5. Reports results per distro

**Output**:
```
Testing rclgo on distros: humble jazzy

=========================================
Testing on ROS 2 humble
=========================================

Pulling ros:humble Docker image...
Running tests in Docker container...
📦 Building rclgo...
🧪 Running tests...
✅ humble: Tests passed!

=========================================
Testing on ROS 2 jazzy
=========================================
...

Test Summary:
  humble: ✅ PASS
  jazzy: ✅ PASS

✅ All tests passed across all distros! 🎉
```

---

## Common Workflows

### Starting a New Feature

```bash
# 1. Create feature branch
./scripts/git-rclgo.sh branch-from humble

# 2. Make changes, commit frequently
git add -p
git commit -m "feat(params): add validation"

# 3. Test locally
go test ./...

# 4. Push and create PR
git push -u origin feature/param-validation
gh pr create --base humble --fill
```

### Syncing to Another Distro

```bash
# After feature merged to humble, sync to jazzy
./scripts/sync-feature.sh humble jazzy HEAD~3..HEAD

# Follow prompts, test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# Push and create PR
git push -u origin sync/humble-to-jazzy-<timestamp>
gh pr create --base jazzy --title "sync: param validation from humble" --fill
```

### Testing Across All Distros

```bash
# Quick check before creating PR
./scripts/test-all-distros.sh

# Or test specific distro
./scripts/test-all-distros.sh --distro=jazzy --verbose
```

---

## Tips

### Git Aliases

Add to your `~/.gitconfig` for convenience:

```ini
[alias]
    # rclgo helpers
    rclgo = !bash /path/to/rclgo/scripts/git-rclgo.sh
    distro = !bash /path/to/rclgo/scripts/git-rclgo.sh which-distro
    qs = !bash /path/to/rclgo/scripts/git-rclgo.sh quick-save

    # General shortcuts
    st = status -sb
    co = checkout
    br = branch
    ci = commit
    unstage = reset HEAD --
```

Then use:
```bash
git distro        # Show current distro
git qs "WIP"      # Quick save
git rclgo oops    # Undo last commit
```

### Pre-commit Hooks

The scripts work well with pre-commit hooks:

```bash
# Install pre-commit framework
pip install pre-commit
pre-commit install

# Hooks will run automatically on commit
# To run manually:
pre-commit run --all-files
```

### Environment Setup

For faster workflows, source ROS in your shell profile:

```bash
# Add to ~/.bashrc or ~/.zshrc
alias ros-humble='source /opt/ros/humble/setup.bash'
alias ros-jazzy='source /opt/ros/jazzy/setup.bash'

# Then use:
ros-humble
go test ./...
```

---

## Troubleshooting

### "Permission denied" when running scripts

Make scripts executable:
```bash
chmod +x scripts/*.sh
```

### Docker permission issues

Add your user to docker group:
```bash
sudo usermod -aG docker $USER
# Log out and back in
```

### Cherry-pick conflicts during sync

See `sync-feature.sh` section above for conflict resolution.

### Tests fail on specific distro

Check ROS environment:
```bash
source /opt/ros/<distro>/setup.bash
echo $ROS_DISTRO  # Should match distro
go test ./... -v  # Verbose output
```

---

## Contributing

When adding new scripts:
1. Make them executable: `chmod +x scripts/new-script.sh`
2. Add to this README
3. Add to `.gitignore` exceptions if needed
4. Test on both distros

---

**See also**:
- [CONTRIBUTING.md](../CONTRIBUTING.md) - Full development workflow
- [docs/BRANCHING.md](../docs/BRANCHING.md) - Branching strategy details
- [docs/RELEASING.md](../docs/RELEASING.md) - Release process
