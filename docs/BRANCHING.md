# rclgo Branching Strategy

This document describes the branching model used in rclgo to support multiple ROS 2 distros.

## Table of Contents

- [Overview](#overview)
- [Branch Types](#branch-types)
- [Workflows](#workflows)
- [Examples](#examples)
- [FAQ](#faq)

## Overview

rclgo uses a **multi-distro branching model** where each ROS 2 distribution has its own long-lived branch. This approach allows:

- Independent development per distro (different API versions, CGO bindings)
- Selective feature porting between distros
- Distro-specific bug fixes
- Clear separation of concerns

### Key Principles

1. **Each distro is independent**: `humble` and `jazzy` branches can diverge
2. **Features sync when applicable**: Most Go code works across distros
3. **CGO/generated code is distro-specific**: Different RCL versions require different bindings
4. **Primary distro drives development**: `humble` is currently primary, but this can change

## Branch Types

### Long-Lived Branches

#### Distro Branches
- **Names**: `humble`, `jazzy`, (future: `iron`, `rolling`)
- **Purpose**: Main development branch for each ROS 2 distro
- **Protection**: ✅ Protected, requires PRs
- **Lifetime**: As long as distro is supported

**Current distros**:
- `humble`: Primary development branch (ROS 2 Humble)
- `jazzy`: Jazzy-specific branch (ROS 2 Jazzy)

#### Master Branch
- **Name**: `master`
- **Purpose**: Points to the current primary distro
- **Protection**: ✅ Protected
- **Behavior**: Fast-forward-only pointer to primary distro (currently `humble`)

```bash
# Master tracks humble (current setup)
master → humble

# In the future, when jazzy becomes primary:
master → jazzy
```

### Work Branches

#### Feature Branches
- **Pattern**: `feature/<descriptive-name>`
- **Base**: Target distro (usually `humble`)
- **Purpose**: New features, enhancements
- **Lifetime**: Until merged
- **Examples**:
  - `feature/logging-api`
  - `feature/param-yaml-loader`
  - `feature/qos-validation`

#### Fix Branches
- **Pattern**: `fix/<descriptive-name>`
- **Base**: Affected distro
- **Purpose**: Bug fixes
- **Lifetime**: Until merged
- **Examples**:
  - `fix/nil-pointer-crash`
  - `fix/jazzy-cgo-binding`
  - `fix/memory-leak`

#### Sync Branches
- **Pattern**: `sync/<from-distro>-to-<to-distro>`
- **Base**: Target distro
- **Purpose**: Port features/fixes between distros
- **Lifetime**: Until merged
- **Examples**:
  - `sync/humble-to-jazzy`
  - `sync/jazzy-to-humble-bugfix`

#### Experiment Branches
- **Pattern**: `experiment/<descriptive-name>`
- **Base**: Any distro
- **Purpose**: Experimental work, prototypes
- **Lifetime**: Until conclusion (merge, discard, or evolve into feature)
- **Examples**:
  - `experiment/action-server-redesign`
  - `experiment/zero-copy-messages`

## Workflows

### Workflow 1: Cross-Distro Feature

Most features work across all distros with minimal changes.

```
humble (primary)
  │
  ├─ feature/awesome-thing ──┐
  │                          │ PR #123
  │◄─────────────────────────┘
  │
  │ (after merge)
  │
jazzy
  │
  ├─ sync/humble-to-jazzy ───┐
  │   (cherry-pick from #123) │ PR #124
  │◄───────────────────────────┘
```

**Steps**:
1. Create `feature/awesome-thing` from `humble`
2. Develop, test, commit
3. PR to `humble`, get reviewed, merge
4. Create `sync/humble-to-jazzy` branch
5. Cherry-pick commits from feature
6. Test on jazzy environment
7. PR to `jazzy`, merge

### Workflow 2: Distro-Specific Fix

Some changes only apply to one distro.

```
jazzy
  │
  ├─ fix/jazzy-api-issue ────┐
  │                           │ PR #125
  │◄──────────────────────────┘
  │
  (no sync needed - distro-specific)
```

**Steps**:
1. Create `fix/jazzy-api-issue` from `jazzy`
2. Fix distro-specific issue
3. PR to `jazzy` with `distro-specific` label
4. Merge (no sync to other distros)

### Workflow 3: Critical Bug Affecting All Distros

Critical bugs need immediate fixing across all distros.

```
humble
  │
  ├─ fix/critical-bug ───────┐
  │                          │ PR #126
  │◄─────────────────────────┘
  │
jazzy
  │
  ├─ fix/critical-bug-jazzy ─┐
  │   (cherry-pick #126)      │ PR #127
  │◄───────────────────────────┘
```

**Steps**:
1. Fix on oldest supported distro first (`humble`)
2. Test thoroughly
3. PR to `humble`, fast-track review
4. **Immediately** create sync branches for other distros
5. Test on each distro
6. PR and merge to all distros

### Workflow 4: Switching Primary Distro

When a new ROS 2 distro becomes primary (e.g., `jazzy` replaces `humble`):

```bash
# Update master to point to new primary distro
git checkout master
git fetch origin jazzy
git reset --hard origin/jazzy
git push origin master --force-with-lease

# Announce in README and documentation
```

**When to switch**:
- New distro is stable and widely adopted
- Most active development moves to new distro
- Older distro enters maintenance mode

## Examples

### Example 1: Adding a New Feature

```bash
# 1. Start from primary distro
git checkout humble
git pull origin humble

# 2. Create feature branch
git checkout -b feature/logger-filtering
# Or use helper: ./scripts/git-rclgo.sh branch-from humble

# 3. Develop feature
# ... make changes ...
git add pkg/rclgo/logging.go
git commit -m "feat(logging): add severity level filtering"

# 4. Push and create PR
git push -u origin feature/logger-filtering
gh pr create --base humble --fill

# 5. After merge, sync to jazzy
git checkout jazzy
git pull origin jazzy
git checkout -b sync/humble-to-jazzy

# Cherry-pick the feature commits
git cherry-pick <commit-hash>

# 6. Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# 7. PR to jazzy
git push -u origin sync/humble-to-jazzy
gh pr create --base jazzy --title "sync: logger filtering from humble" --fill
```

### Example 2: Fixing a Jazzy-Specific Bug

```bash
# 1. Branch from jazzy
git checkout jazzy
git pull origin jazzy
git checkout -b fix/jazzy-rcl-binding

# 2. Fix the bug
# ... edit CGO code ...
git add pkg/rclgo/rcl.go
git commit -m "fix(jazzy): update RCL binding for jazzy API"

# 3. Test on jazzy
source /opt/ros/jazzy/setup.bash
go test ./...

# 4. PR to jazzy with distro-specific label
git push -u origin fix/jazzy-rcl-binding
gh pr create --base jazzy --label "distro-specific" --fill

# No sync needed - this is jazzy-specific!
```

### Example 3: Using Helper Scripts

```bash
# Create feature branch interactively
./scripts/git-rclgo.sh branch-from humble
# Follow prompts...

# Sync commits between distros
./scripts/sync-feature.sh humble jazzy HEAD~3..HEAD
# Script handles cherry-picking, testing prompts, etc.

# Check which distro context you're in
./scripts/git-rclgo.sh which-distro
```

## FAQ

### Q: Which distro should I develop on?

**A**: Use the **primary distro** (`humble` currently) unless:
- You're fixing a distro-specific bug
- The feature only makes sense for a specific distro
- You're porting from another distro

### Q: How do I know if a feature needs syncing?

**A**: Sync if:
- ✅ Feature is pure Go code (logic, APIs)
- ✅ Feature uses RCL APIs that exist in all distros
- ✅ Tests pass on multiple distros

Don't sync if:
- ❌ Feature uses distro-specific RCL APIs
- ❌ Feature depends on distro-specific message definitions
- ❌ Feature is a CGO binding for a distro-specific API version

### Q: What if I need to support a new distro?

**A**: Steps to add a new distro (e.g., `iron`):

1. Create distro branch from humble:
   ```bash
   git checkout -b iron humble
   ```

2. Update for new distro:
   - Regenerate messages: `make generate`
   - Update CGO bindings if needed
   - Test: `source /opt/ros/iron/setup.bash && go test ./...`

3. Push distro branch:
   ```bash
   git push origin iron
   ```

4. Update CI to include new distro:
   - Edit `.github/workflows/ci.yml`
   - Add `iron` to matrix

5. Document in README and ROADMAP

### Q: Can I delete my feature branch after merging?

**A**: Yes! After your PR is merged:
```bash
git checkout humble
git pull origin humble
git branch -d feature/my-feature
git push origin --delete feature/my-feature
```

GitHub will also prompt you to delete the branch after merge.

### Q: What if cherry-picking creates conflicts?

**A**: Resolve manually:

```bash
# During sync
git cherry-pick <commit>
# CONFLICT: ...

# 1. Fix conflicts in your editor
# 2. Mark as resolved
git add <fixed-files>

# 3. Continue cherry-pick
git cherry-pick --continue

# Or abort if needed
git cherry-pick --abort
```

### Q: How do I update my feature branch with latest changes from humble?

**A**: Rebase regularly:

```bash
git checkout feature/my-feature
git fetch origin humble
git rebase origin/humble

# If conflicts occur, resolve and continue
git rebase --continue
```

### Q: Should master always equal humble?

**A**: Not necessarily. `master` points to the **primary distro**:
- Currently: `master` → `humble`
- Future: `master` → `jazzy` (when jazzy becomes primary)

This allows users cloning the repo to get the most relevant version by default.

### Q: What about release branches?

**A**: See `docs/RELEASING.md` for release workflow. Short version:
- Releases are tagged on distro branches: `v0.4.1-humble`
- No separate release branches (distro branches serve this purpose)
- `master` can point to latest release tag if desired

---

**Next**: See [CONTRIBUTING.md](../CONTRIBUTING.md) for day-to-day development workflow.
