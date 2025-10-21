# rclgo Daily Workflow Guide

This document provides practical, day-to-day workflow guidance for working with rclgo's multi-distro git setup.

## Table of Contents

- [Starting Your Day](#starting-your-day)
- [Working on Features](#working-on-features)
- [Creating Pull Requests](#creating-pull-requests)
- [Syncing to Other Distros](#syncing-to-other-distros)
- [Releasing Versions](#releasing-versions)
- [Common Commands](#common-commands)
- [Typical Week Example](#typical-week-example)
- [Golden Rules](#golden-rules)

---

## Starting Your Day

```bash
# 1. Update your primary distro
git checkout humble
git pull origin humble

# 2. Check what you were working on
git branch  # See your feature branches
./scripts/git-rclgo.sh which-distro  # Get context

# 3. Continue existing work OR start new feature
git checkout feature/my-feature  # Continue existing
# OR
./scripts/git-rclgo.sh branch-from humble  # Start new
```

---

## Working on Features

### Starting a New Feature

**Using the helper script (recommended):**
```bash
./scripts/git-rclgo.sh branch-from humble
# Interactive prompts:
#   - Feature name: "logging-api"
#   - Type: 1) feature  2) fix  3) experiment
# Creates: feature/logging-api from humble
```

**Manual approach:**
```bash
git checkout humble
git pull origin humble
git checkout -b feature/logging-api
```

### Making Changes (Every 15-30 Minutes)

```bash
# 1. Make changes in your editor
# ... edit files ...

# 2. Review what changed
git status
git diff

# 3. Stage changes (review each chunk)
git add -p  # Interactive staging - recommended!
# OR
git add <specific-files>

# 4. Commit with meaningful message
git commit
# The prepare-commit-msg hook will help with format:
# feat(logging): add severity level filtering
#
# Implements LogSeverity enum and adds filtering...

# 5. Keep working, repeat every 15-30 min
```

**Commit Frequency Guidelines:**

| Time Spent | Action | Result |
|------------|--------|--------|
| 15 min | Added severity enum | ✅ Commit |
| 20 min | Implemented filtering logic | ✅ Commit |
| 10 min | Added tests | ✅ Commit |
| 3 hours | Everything done | ❌ Too infrequent! |

### Quick Save (When Interrupted)

```bash
# Need to switch contexts quickly?
./scripts/git-rclgo.sh quick-save "WIP: debugging param validation"
# Commits all changes with timestamp

# Or manually
git add -A
git commit -m "WIP: quick save before meeting"
```

### Recovering from Mistakes

**Undo last commit (keep changes):**
```bash
./scripts/git-rclgo.sh oops
# Now you can fix and recommit

# Or manually
git reset --soft HEAD~1
```

**Committed to wrong branch?**
```bash
# Move commits to correct branch
git checkout correct-branch
git cherry-pick <commit-hash>
```

**Discard uncommitted changes:**
```bash
git restore <file>
# Or all changes
git restore .
```

**Find lost commits:**
```bash
git reflog  # Your git time machine
git cherry-pick <lost-commit-hash>
```

---

## Creating Pull Requests

### When Your Feature is Ready

```bash
# 1. Make sure you're up to date
git checkout humble
git pull origin humble
git checkout feature/logging-api
git rebase humble  # Incorporate latest changes

# 2. Push to remote (pre-push hook runs tests automatically)
git push -u origin feature/logging-api
# This might take a few minutes - the hook is running tests!

# 3. Create PR
gh pr create --base humble --fill
# The PR template will guide you through the checklist

# 4. Fill out the PR template
#   - Check type of change
#   - Mark target distro (humble)
#   - Note if it needs syncing to jazzy
#   - Describe what changed
#   - Confirm tests pass
```

### If Pre-push Hook Blocks You

```bash
# Hook runs tests and fails
❌ Tests failed! Push aborted.
   Fix the tests or use 'git push --no-verify' to skip this check.
```

**Option 1: Fix the tests (recommended)**
```bash
source /opt/ros/humble/setup.bash
go test ./...  # See what's failing
# ... fix tests ...
git add <test-files>
git commit -m "fix: correct failing tests"
git push
```

**Option 2: Skip if it's a false positive**
```bash
git push --no-verify
# Use sparingly - usually tests fail for a reason!
```

### After Creating PR

1. **Self-review on GitHub** - Catch your own bugs before they merge
2. **Wait for CI** - GitHub Actions will run automatically
3. **Verify CI passes** - Look for green checkmarks
4. **Merge when ready** - You can merge your own PRs as solo dev
5. **Delete the branch** - GitHub will prompt after merge

**Updating your PR after feedback:**
```bash
# Make changes based on review
# ... edit files ...
git add <files>
git commit -m "fix: address review comments"
git push  # Updates the PR automatically
```

---

## Syncing to Other Distros

### After Merging to Humble

Most features should be synced to jazzy to maintain feature parity.

**Using the helper script (recommended):**
```bash
# 1. Sync from humble to jazzy
./scripts/sync-feature.sh humble jazzy HEAD~5..HEAD
# The script will:
#   - Show commits to sync
#   - Ask for confirmation
#   - Create sync branch: sync/humble-to-jazzy-<timestamp>
#   - Cherry-pick commits
#   - Offer to run tests

# 2. Already checked out on sync branch

# 3. Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# 4. Push and create PR
git push -u origin sync/humble-to-jazzy-<timestamp>
gh pr create --base jazzy --title "sync: logging-api from humble" --fill

# 5. Merge to jazzy
```

**Using git-rclgo helper:**
```bash
./scripts/git-rclgo.sh sync humble jazzy HEAD~3..HEAD
# Similar to sync-feature.sh but more interactive
```

**Manual approach (if you prefer):**
```bash
git checkout jazzy
git pull origin jazzy
git checkout -b sync/humble-to-jazzy

# Cherry-pick the commits
git cherry-pick <commit-range>

# Test, push, PR
source /opt/ros/jazzy/setup.bash
go test ./...
git push -u origin sync/humble-to-jazzy
gh pr create --base jazzy --fill
```

### Handling Sync Conflicts

If cherry-picking creates conflicts:

```bash
# CONFLICT during cherry-pick
git status  # See conflicted files

# 1. Fix conflicts in your editor
vim <conflicted-file>

# 2. Mark as resolved
git add <fixed-files>

# 3. Continue cherry-pick
git cherry-pick --continue

# Or abort if needed
git cherry-pick --abort
```

### Distro-Specific Changes (No Sync Needed)

If your change is distro-specific (e.g., jazzy CGO binding fix):

```bash
# Branch from target distro
git checkout jazzy
git checkout -b fix/jazzy-cgo-issue

# Make changes, commit, PR
git push -u origin fix/jazzy-cgo-issue
gh pr create --base jazzy --label "distro-specific" --fill

# No sync needed - this only applies to jazzy
```

---

## Releasing Versions

### Preparing a Release

```bash
# 1. Ensure you're on the distro branch
git checkout humble
git pull origin humble

# 2. Update VERSION file
echo "0.6.0" > VERSION

# 3. Update CHANGELOG.md
vim CHANGELOG.md
```

**CHANGELOG.md format:**
```markdown
## v0.6.0 - 2025-01-21

### Features
- Add logging severity filtering
- Implement param validation

### Bug Fixes
- Fix nil pointer in param manager

### Breaking Changes
- None
```

```bash
# 4. Commit version bump
git add VERSION CHANGELOG.md
git commit -m "chore: bump VERSION to 0.6.0 for v0.6.0-humble"

# 5. Run final checks
source /opt/ros/humble/setup.bash
go test ./... -timeout 10m
go build ./...
make lint  # If golangci-lint is installed
```

### Creating the Release

```bash
# 6. Create annotated tag
git tag -a v0.6.0-humble -m "Release v0.6.0 for ROS 2 Humble

Features:
- Add logging severity filtering
- Implement param validation

Bug Fixes:
- Fix nil pointer in param manager

See CHANGELOG.md for details."

# 7. Push commit and tag
git push origin humble
git push origin v0.6.0-humble

# 8. Create GitHub release
gh release create v0.6.0-humble \
    --title "rclgo v0.6.0 for ROS 2 Humble" \
    --notes-file CHANGELOG.md \
    --target humble
```

### Releasing on Other Distros

If jazzy has feature parity:

```bash
# Repeat process for jazzy
git checkout jazzy
git pull origin jazzy

echo "0.6.0" > VERSION
vim CHANGELOG.md  # Update for jazzy

git add VERSION CHANGELOG.md
git commit -m "chore: bump VERSION to 0.6.0 for v0.6.0-jazzy"

git tag -a v0.6.0-jazzy -m "Release v0.6.0 for ROS 2 Jazzy"
git push origin jazzy v0.6.0-jazzy

gh release create v0.6.0-jazzy \
    --title "rclgo v0.6.0 for ROS 2 Jazzy" \
    --notes-file CHANGELOG.md \
    --target jazzy
```

**See [RELEASING.md](RELEASING.md) for complete release documentation.**

---

## Common Commands

### Branch Management

```bash
# Create feature from humble
./scripts/git-rclgo.sh branch-from humble

# Check which distro you're on
./scripts/git-rclgo.sh which-distro

# Switch branches
git checkout humble
git checkout feature/my-feature

# List all branches
git branch
git branch -a  # Include remote branches

# Delete merged branch
git branch -d feature/old-feature
```

### Committing

```bash
# Interactive staging (review each change)
git add -p

# Stage specific files
git add <file1> <file2>

# Commit with message
git commit -m "feat(scope): description"

# Commit with interactive editor (better for details)
git commit

# Quick save
./scripts/git-rclgo.sh quick-save "WIP message"

# Undo last commit (keep changes)
./scripts/git-rclgo.sh oops
# Or manually
git reset --soft HEAD~1

# Amend last commit (fix typo, add forgotten file)
git add <forgotten-file>
git commit --amend
```

### Pushing & Pull Requests

```bash
# Push (runs tests automatically via pre-push hook)
git push

# First push of new branch
git push -u origin feature/my-feature

# Push without running tests (use sparingly)
git push --no-verify

# Create PR
gh pr create --base humble --fill

# Create PR with specific title
gh pr create --base humble --title "feat: add awesome feature" --fill

# Update PR (after making changes)
git push  # Same branch, updates PR
```

### Syncing Between Distros

```bash
# Sync from humble to jazzy (interactive)
./scripts/sync-feature.sh humble jazzy HEAD~5..HEAD

# Sync specific commit range
./scripts/sync-feature.sh humble jazzy abc123..def456

# Sync via git-rclgo
./scripts/git-rclgo.sh sync humble jazzy HEAD~5..HEAD

# Show commits that differ between distros
git log origin/humble..origin/jazzy  # In jazzy, not humble
git log origin/jazzy..origin/humble  # In humble, not jazzy
```

### Testing

```bash
# Local tests (current distro)
source /opt/ros/humble/setup.bash
go test ./...

# Fast tests only
go test -short ./...

# Specific package
go test ./pkg/rclgo/params/...

# With coverage
go test -v -race -coverprofile=coverage.txt ./...

# All distros via Docker
./scripts/test-all-distros.sh

# Specific distro
./scripts/test-all-distros.sh --distro=jazzy

# Verbose output
./scripts/test-all-distros.sh --verbose
```

### Viewing History

```bash
# Recent commits
git log --oneline -10

# Graphical view
git log --oneline --graph --decorate --all

# Changes in a file
git log -p <file>

# See what changed in a commit
git show <commit-hash>

# Find when something changed
git log -S "searchterm" --source --all

# Reflog (see all ref changes)
git reflog
```

### Updating Your Branch

```bash
# Update with latest from humble
git checkout feature/my-feature
git fetch origin humble
git rebase origin/humble

# If conflicts occur
# 1. Fix conflicts in editor
# 2. git add <resolved-files>
# 3. git rebase --continue

# Or abort rebase
git rebase --abort
```

---

## Typical Week Example

### Monday Morning
```bash
git checkout humble && git pull origin humble
./scripts/git-rclgo.sh branch-from humble
# Create feature/param-validation
# Work 2 hours, make 4 commits
git push -u origin feature/param-validation
gh pr create --base humble --fill
```

### Tuesday
```bash
git checkout feature/param-validation
# Work all day, make 8 commits
git push  # Update PR
# Address review comments
git commit -m "fix: address review feedback"
git push
```

### Wednesday
```bash
# PR approved, merge via GitHub UI
git checkout humble && git pull origin humble
git branch -d feature/param-validation

# Start new feature
./scripts/git-rclgo.sh branch-from humble
# Create feature/qos-events
# Work, commit frequently, push
gh pr create --base humble --fill
```

### Thursday
```bash
# Sync Monday's feature to jazzy
./scripts/sync-feature.sh humble jazzy abc123..def456
# Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...
# Push and create PR
git push -u origin sync/humble-to-jazzy-<timestamp>
gh pr create --base jazzy --title "sync: param-validation from humble" --fill
```

### Friday
```bash
# Bug discovered, quick fix
git checkout humble
git checkout -b fix/memory-leak
# Fix in 30 min, commit, push
git push -u origin fix/memory-leak
gh pr create --base humble --fill
# Fast-track review, merge

# Sync fix to jazzy immediately
./scripts/sync-feature.sh humble jazzy HEAD~1..HEAD
git push -u origin sync/humble-to-jazzy-<timestamp>
gh pr create --base jazzy --fill

# All done, clean up
git checkout humble && git pull
git branch -d fix/memory-leak
```

---

## Golden Rules

### The 7 Commandments of rclgo Development

1. **Commit every 15-30 minutes**
   - Small, frequent commits are better than large infrequent ones
   - Each commit should compile and ideally pass tests

2. **Never work directly on humble/jazzy/master**
   - Always use feature branches
   - Pre-commit hooks will prevent this, but get in the habit

3. **Always create pull requests**
   - Even for solo work (CI catches issues)
   - Self-review your PRs on GitHub

4. **One logical change per commit**
   - If you're using "and" in the commit message, split it
   - Bad: "fix bug and add feature and update docs"
   - Good: Three separate commits

5. **Test before pushing**
   - Pre-push hook helps, but verify manually too
   - `source /opt/ros/humble/setup.bash && go test ./...`

6. **Sync features to jazzy**
   - Keep distros in sync for cross-distro features
   - Mark distro-specific changes clearly

7. **Use the helper scripts**
   - They automate the boring parts
   - `git-rclgo.sh`, `sync-feature.sh`, `test-all-distros.sh`

---

## Quick Reference Card

| Task | Command |
|------|---------|
| **Start new feature** | `./scripts/git-rclgo.sh branch-from humble` |
| **Check context** | `./scripts/git-rclgo.sh which-distro` |
| **Stage changes** | `git add -p` (interactive) |
| **Commit** | `git commit` |
| **Quick save** | `./scripts/git-rclgo.sh quick-save "msg"` |
| **Undo commit** | `./scripts/git-rclgo.sh oops` |
| **Push + create PR** | `git push && gh pr create --base humble --fill` |
| **Update PR** | `git push` (on same branch) |
| **Sync to jazzy** | `./scripts/sync-feature.sh humble jazzy HEAD~5..HEAD` |
| **Test locally** | `source /opt/ros/humble/setup.bash && go test ./...` |
| **Test all distros** | `./scripts/test-all-distros.sh` |
| **View history** | `git log --oneline --graph --all` |
| **Update branch** | `git rebase origin/humble` |
| **Release** | See [RELEASING.md](RELEASING.md) |

---

## Workflow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     START OF DAY                            │
│  git checkout humble && git pull origin humble              │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              CREATE FEATURE BRANCH                          │
│  ./scripts/git-rclgo.sh branch-from humble                  │
│  → Creates: feature/awesome-thing                           │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              WORK & COMMIT CYCLE                            │
│  ┌──────────────────────────────────────────────┐          │
│  │ 1. Make changes (15-30 min)                  │          │
│  │ 2. git add -p                                │          │
│  │ 3. git commit -m "feat: thing"               │          │
│  │ 4. Repeat ──────────────────────┐            │          │
│  └─────────────────────────────────┘            │          │
│                   │                              │          │
│                   └──────────────────────────────┘          │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              PUSH & CREATE PR                               │
│  git push -u origin feature/awesome-thing                   │
│  gh pr create --base humble --fill                          │
│  → Pre-push hook runs tests                                 │
│  → CI runs on GitHub                                        │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              REVIEW & MERGE                                 │
│  Self-review on GitHub                                      │
│  Wait for CI ✅                                             │
│  Merge PR                                                   │
│  Delete feature branch                                      │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              SYNC TO JAZZY                                  │
│  ./scripts/sync-feature.sh humble jazzy HEAD~5..HEAD        │
│  → Creates: sync/humble-to-jazzy-<timestamp>                │
│  → Cherry-picks commits                                     │
│  source /opt/ros/jazzy/setup.bash && go test ./...          │
│  git push && gh pr create --base jazzy --fill               │
│  Merge to jazzy                                             │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              CLEAN UP                                       │
│  git checkout humble && git pull                            │
│  Ready for next feature!                                    │
└─────────────────────────────────────────────────────────────┘
```

---

## Troubleshooting

### Problem: Pre-commit hook rejects my commit

**Solution:**
```bash
# Check what failed
pre-commit run --all-files

# Fix issues (usually formatting)
go fmt ./...

# Re-commit
git add -A
git commit
```

### Problem: Pre-push hook blocks my push

**Solution:**
```bash
# See what tests failed
source /opt/ros/humble/setup.bash
go test ./... -v

# Fix tests, then push
# Or skip if false positive
git push --no-verify
```

### Problem: Cherry-pick conflicts during sync

**Solution:**
```bash
# Fix conflicts in editor
vim <conflicted-file>

# Mark resolved
git add <fixed-file>

# Continue
git cherry-pick --continue
```

### Problem: Accidentally committed to humble

**Solution:**
```bash
# Move commits to a feature branch
git branch feature/oops humble  # Create branch at current humble
git reset --hard origin/humble  # Reset humble to remote
git checkout feature/oops  # Continue work here
```

### Problem: Lost commits after reset

**Solution:**
```bash
# Find lost commits
git reflog

# Cherry-pick them back
git cherry-pick <commit-hash>
```

---

## Additional Resources

- **[CONTRIBUTING.md](../CONTRIBUTING.md)** - Complete contribution guidelines
- **[BRANCHING.md](BRANCHING.md)** - Detailed branching strategy
- **[RELEASING.md](RELEASING.md)** - Release process documentation
- **[scripts/README.md](../scripts/README.md)** - Helper scripts documentation
- **[ROADMAP.md](../ROADMAP.md)** - Feature parity tracking

---

**Remember**: The tools are here to help you. Don't fight them - they're preventing mistakes and saving you time! 🚀
