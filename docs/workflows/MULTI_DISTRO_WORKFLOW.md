# rclgo Daily Workflow Guide (Simplified)

This document provides practical, day-to-day workflow guidance for working with rclgo's multi-distro git setup.

**Key Principle**: Feature branches stay LOCAL. Only distro branches (humble, jazzy) get pushed to GitHub.

## Table of Contents

- [Starting Your Day](#starting-your-day)
- [Working on Features](#working-on-features)
- [Merging and Pushing](#merging-and-pushing)
- [Syncing to Other Distros](#syncing-to-other-distros)
- [Common Commands](#common-commands)
- [Master Branch](#master-branch)
- [Releases](#releases)
- [Golden Rules](#golden-rules)

---

## Starting Your Day

```bash
# 1. Update your primary distro
git checkout humble
git pull origin humble

# 2. Check what you were working on
git branch  # See your local feature branches

# 3. Continue existing work OR start new feature
git checkout feature/my-feature  # Continue existing
# OR
git checkout -b feature/new-thing  # Start new (stays local!)
```

---

## Working on Features

### Create Feature Branch (Local Only)

```bash
git checkout humble
git checkout -b feature/logging-api
# This branch will NEVER be pushed to GitHub
```

### Make Changes (Every 15-30 Minutes)

```bash
# 1. Edit files
# ... make changes ...

# 2. Review changes
git status
git diff

# 3. Stage changes
git add -p  # Interactive staging (recommended)
# OR
git add <specific-files>

# 4. Commit with meaningful message
git commit -m "feat(logging): add severity filtering"

# 5. Repeat - commit often!
```

**Commit Frequency**:
- ✅ Every 15-30 minutes of focused work
- ✅ When a logical unit is complete
- ✅ Before switching tasks
- ❌ NOT once at the end of the day

### Quick Save

```bash
# Need to switch contexts quickly?
git add -A
git commit -m "WIP: debugging param validation"
```

### Undo Last Commit

```bash
# Made a mistake in your last commit?
git reset --soft HEAD~1
# Changes are back in staging, fix and recommit
```

---

## Merging and Pushing

**IMPORTANT**: Feature branches stay local. Only merge and push distro branches.

### When Feature is Complete

```bash
# 1. Make sure you're up to date
git checkout humble
git pull origin humble

# 2. Merge your feature
git merge feature/logging-api
# Resolves conflicts if any

# 3. Test on humble
source /opt/ros/humble/setup.bash
go test ./...
go build ./...

# 4. Push to GitHub (humble only, NOT the feature branch)
git push origin humble

# 5. Clean up local feature branch
git branch -d feature/logging-api
```

### If You Try to Push a Feature Branch

The pre-push hook will block you:

```bash
git checkout feature/my-work
git push

# Hook blocks with:
❌ ERROR: You're trying to push a work branch: feature/my-work

Work branches should stay local. Only push to distro branches (humble, jazzy).

Workflow:
  1. Work on your feature branch locally
  2. When ready: git checkout humble
  3. Then: git merge feature/my-work
  4. Then: git push origin humble
```

To override (not recommended): `git push --no-verify`

---

## Syncing to Other Distros

### After Merging to Humble

Sync features to jazzy to maintain feature parity:

```bash
# 1. Checkout jazzy
git checkout jazzy
git pull origin jazzy

# 2. Cherry-pick or merge from humble
# Option A: Cherry-pick specific commits
git cherry-pick <commit-hash>

# Option B: Merge from humble
git merge humble
# Resolve conflicts if needed

# 3. Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# 4. Push jazzy
git push origin jazzy
```

### For Distro-Specific Changes

```bash
# Work directly on target distro branch
git checkout jazzy
git checkout -b fix/jazzy-specific-issue

# Make changes, commit
git add <files>
git commit -m "fix(jazzy): CGO binding for jazzy API"

# Merge to jazzy
git checkout jazzy
git merge fix/jazzy-specific-issue

# Push
git push origin jazzy

# Clean up
git branch -d fix/jazzy-specific-issue

# No sync to humble needed - this is jazzy-specific
```

---

## Common Commands

### Branch Management

```bash
# Create local feature branch
git checkout -b feature/my-thing

# List branches
git branch                    # Local branches
git branch -a                 # All branches (including remote)

# Switch branches
git checkout humble
git checkout feature/my-thing

# Delete local branch
git branch -d feature/old-feature
git branch -D feature/force-delete  # Force delete
```

### Committing

```bash
# Interactive staging (review each change)
git add -p

# Stage specific files
git add <file1> <file2>

# Commit
git commit -m "feat(scope): description"

# Amend last commit
git commit --amend

# Undo last commit (keep changes)
git reset --soft HEAD~1
```

### Merging

```bash
# Merge feature to humble
git checkout humble
git merge feature/my-feature

# Abort merge if conflicts are too complex
git merge --abort

# After resolving conflicts
git add <resolved-files>
git commit  # Completes the merge
```

### Pushing

```bash
# Push distro branch (allowed)
git checkout humble
git push origin humble

# Push feature branch (BLOCKED by hook)
git checkout feature/my-work
git push  # ❌ Will fail

# Override hook (not recommended)
git push --no-verify
```

### Viewing History

```bash
# Recent commits
git log --oneline -10

# Graphical view
git log --oneline --graph --all

# See what changed in a commit
git show <commit-hash>

# Find when something changed
git log -S "search term" --all
```

### Recovering from Mistakes

```bash
# See all ref changes (your time machine)
git reflog

# Undo last commit (keep changes)
git reset --soft HEAD~1

# Discard all uncommitted changes
git restore .

# Restore specific file
git restore <file>

# Recover lost commits
git reflog  # Find the commit
git cherry-pick <commit-hash>
```

---

## Master Branch

**master** is a pointer to the primary distro branch (currently **humble**). It is NOT a working branch.

### Updating Master

Only update master via fast-forward merge from humble:

```bash
# After pushing to humble
git checkout master
git merge --ff-only humble
git push origin master

# This keeps master pointing to humble
```

**Do NOT**:
- ❌ Create feature branches from master
- ❌ Commit directly to master
- ❌ Merge to master (use --ff-only)

**Why master exists**: It allows users to clone/install the "latest stable" version without knowing which distro is primary.

---

## Releases

### Creating a Release

Releases are per-distro and use version tags:

```bash
# 1. Update VERSION file on humble
git checkout humble
echo "0.6.0" > VERSION
git add VERSION
git commit -m "chore: bump version to 0.6.0"

# 2. Create annotated tag
git tag -a v0.6.0-humble -m "Release v0.6.0 for ROS 2 Humble"

# 3. Push humble and tag
git push origin humble v0.6.0-humble

# 4. Update master to point to new humble
git checkout master
git merge --ff-only humble
git push origin master

# 5. If jazzy is in sync, tag it too
git checkout jazzy
git tag -a v0.6.0-jazzy -m "Release v0.6.0 for ROS 2 Jazzy"
git push origin jazzy v0.6.0-jazzy
```

### Release Naming Convention

- `v0.6.0-humble` - Release for ROS 2 Humble
- `v0.6.0-jazzy` - Release for ROS 2 Jazzy
- Distro suffix indicates which ROS 2 version it targets

### Viewing Releases

```bash
# List all tags
git tag -l

# View specific release
git show v0.6.0-humble

# Checkout a specific release
git checkout v0.6.0-humble
```

---

## Golden Rules

### The 5 Commandments

1. **Feature branches stay local**
   - Never push feature/fix/experiment branches to GitHub
   - Only push humble and jazzy (distro branches)
   - Master is updated via fast-forward from humble only

2. **Commit every 15-30 minutes**
   - Small, frequent commits
   - Each commit should build

3. **One logical change per commit**
   - If using "and" in commit message, split it
   - Bad: "fix bug and add feature"
   - Good: Two separate commits

4. **Always merge to distro branch before pushing**
   - Work on feature/my-thing
   - Merge to humble
   - Push humble (not the feature branch)

5. **Test before pushing**
   - `source /opt/ros/humble/setup.bash`
   - `go test ./...`
   - `go build ./...`

---

## Quick Reference Card

| Task | Command |
|------|---------|
| **Start feature** | `git checkout -b feature/name` |
| **Commit** | `git add -p && git commit` |
| **Undo commit** | `git reset --soft HEAD~1` |
| **Merge to humble** | `git checkout humble && git merge feature/name` |
| **Push** | `git push origin humble` (never push feature branches!) |
| **Sync to jazzy** | `git checkout jazzy && git cherry-pick <commit>` |
| **Test** | `source /opt/ros/humble/setup.bash && go test ./...` |
| **View history** | `git log --oneline --graph --all` |

---

## Typical Week Example

### Monday Morning
```bash
git checkout humble && git pull origin humble
git checkout -b feature/param-validation
# Work 2 hours, make 4 commits locally
```

### Tuesday
```bash
git checkout feature/param-validation
# Work all day, make 8 commits locally
```

### Wednesday
```bash
# Feature complete, merge to humble
git checkout humble
git merge feature/param-validation
go test ./...  # Test
git push origin humble  # Push humble, not feature branch
git branch -d feature/param-validation  # Clean up

# Start new feature
git checkout -b feature/qos-events
# Work, commit locally
```

### Thursday
```bash
# Sync Monday's feature to jazzy
git checkout jazzy
git pull origin jazzy
git cherry-pick <commits-from-param-validation>
source /opt/ros/jazzy/setup.bash && go test ./...
git push origin jazzy
```

### Friday
```bash
# Quick bug fix
git checkout humble
git checkout -b fix/memory-leak
# Fix in 30 min, commit locally
git checkout humble
git merge fix/memory-leak
go test ./...
git push origin humble
git branch -d fix/memory-leak
```

---

## Workflow Diagram

```
Local Work (Never Pushed)                   GitHub (Only Distro Branches)
─────────────────────────                   ──────────────────────────────

feature/my-thing (local)
  │
  ├── commit #1
  ├── commit #2
  ├── commit #3
  │
  └── (merge to humble) ─────────────────> humble (pushed)
                                               │
                                               └── (sync) ──────> jazzy (pushed)
```

---

## Troubleshooting

### Problem: Accidentally tried to push feature branch

**Solution**: The hook blocked it! Just merge to humble first:
```bash
git checkout humble
git merge feature/my-work
git push origin humble
```

### Problem: Made changes on humble instead of feature branch

**Solution**: Move commits to a feature branch:
```bash
git branch feature/oops  # Create branch at current position
git reset --hard origin/humble  # Reset humble to remote
git checkout feature/oops  # Continue work here
```

### Problem: Merge conflicts

**Solution**: Resolve manually:
```bash
# During merge
git status  # See conflicted files
# Fix conflicts in editor
git add <resolved-files>
git commit  # Complete the merge

# Or abort
git merge --abort
```

### Problem: Lost commits after reset

**Solution**: Use reflog:
```bash
git reflog  # Find lost commit
git cherry-pick <commit-hash>
```

---

**Remember**: Keep it simple. Feature branches are for local work. Only humble and jazzy go to GitHub! 🚀
