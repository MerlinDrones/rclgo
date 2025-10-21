# Releasing rclgo

This document describes the release process for rclgo across multiple ROS 2 distros.

## Table of Contents

- [Overview](#overview)
- [Versioning Scheme](#versioning-scheme)
- [Release Process](#release-process)
- [Examples](#examples)
- [Hotfix Releases](#hotfix-releases)

## Overview

rclgo uses a **per-distro release model** where each ROS 2 distribution gets its own versioned releases. This allows:

- Independent versioning per distro
- Distro-specific bug fixes without affecting others
- Clear indication of which distro a release targets

## Versioning Scheme

### Version Format

```
vMAJOR.MINOR.PATCH-DISTRO
```

**Examples**:
- `v0.4.1-humble` - Version 0.4.1 for ROS 2 Humble
- `v0.4.1-jazzy` - Version 0.4.1 for ROS 2 Jazzy
- `v1.0.0-humble` - Version 1.0.0 for Humble

### Semantic Versioning

We follow [Semantic Versioning 2.0.0](https://semver.org/):

- **MAJOR**: Breaking API changes
- **MINOR**: New features (backward compatible)
- **PATCH**: Bug fixes (backward compatible)

### Cross-Distro Versioning

- Version numbers can be synchronized across distros for feature parity
- Or independent if distros diverge significantly

**Example: Synchronized versions**:
- `v0.5.0-humble` and `v0.5.0-jazzy` both have same features

**Example: Independent versions**:
- `v0.5.2-humble` (has 2 patches)
- `v0.5.1-jazzy` (has 1 patch)
- Humble got an extra distro-specific bugfix

## Release Process

### Prerequisites

- All tests pass on target distro
- Documentation is up to date
- CHANGELOG has entry for this version
- No open critical bugs for this release

### Step-by-Step Process

#### 1. Prepare the Release

**On distro branch** (e.g., `humble`):

```bash
# Ensure branch is up to date
git checkout humble
git pull origin humble

# Update VERSION file
echo "0.5.0" > VERSION
git add VERSION

# Update CHANGELOG.md
# Add section for v0.5.0 with changes since last release
vim CHANGELOG.md
git add CHANGELOG.md

# Commit version bump
git commit -m "chore: bump VERSION to 0.5.0 for v0.5.0-humble"
```

#### 2. Run Final Checks

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Full test suite
go test ./... -timeout 10m

# Build all packages
go build ./...

# Test examples
for example_dir in examples/*/; do
    cd "$example_dir"
    go build ./...
    cd -
done

# Lint
golangci-lint run --timeout=5m

# Regenerate messages (ensure no drift)
make generate
git diff --exit-code pkg/msgs/
```

#### 3. Create and Push Tag

```bash
# Create annotated tag
git tag -a v0.5.0-humble -m "Release v0.5.0 for ROS 2 Humble

Features:
- Add parameter YAML loading
- Implement QoS validation
- Support external log handlers

Bug Fixes:
- Fix nil pointer in param manager
- Correct timer callback scheduling

See CHANGELOG.md for details."

# Push tag
git push origin v0.5.0-humble

# Push version bump commit
git push origin humble
```

#### 4. Create GitHub Release

```bash
# Using GitHub CLI
gh release create v0.5.0-humble \
    --title "v0.5.0-humble" \
    --notes-file CHANGELOG.md \
    --target humble

# Or use GitHub web UI:
# 1. Go to https://github.com/MerlinDrones/rclgo/releases/new
# 2. Select tag: v0.5.0-humble
# 3. Title: "rclgo v0.5.0 for ROS 2 Humble"
# 4. Copy release notes from CHANGELOG.md
# 5. Attach any binaries (if applicable)
# 6. Publish release
```

#### 5. Release on Other Distros

If feature parity exists, release on other distros:

```bash
# Switch to jazzy
git checkout jazzy
git pull origin jazzy

# Update VERSION
echo "0.5.0" > VERSION

# Update CHANGELOG for jazzy
vim CHANGELOG.md

# Commit and tag
git add VERSION CHANGELOG.md
git commit -m "chore: bump VERSION to 0.5.0 for v0.5.0-jazzy"
git tag -a v0.5.0-jazzy -m "Release v0.5.0 for ROS 2 Jazzy"

# Push
git push origin jazzy
git push origin v0.5.0-jazzy

# Create GitHub release
gh release create v0.5.0-jazzy \
    --title "v0.5.0-jazzy" \
    --notes-file CHANGELOG.md \
    --target jazzy
```

#### 6. Update Documentation

- Update README.md with latest version
- Update ROADMAP.md if features changed status
- Announce release (GitHub Discussions, etc.)

## Examples

### Example 1: Minor Release with Feature Parity

Both Humble and Jazzy get v0.5.0 with same features.

```bash
# Release on humble
git checkout humble
echo "0.5.0" > VERSION
# ... update CHANGELOG ...
git commit -m "chore: bump VERSION to 0.5.0 for v0.5.0-humble"
git tag -a v0.5.0-humble -m "Release v0.5.0"
git push origin humble v0.5.0-humble
gh release create v0.5.0-humble --target humble

# Release on jazzy
git checkout jazzy
echo "0.5.0" > VERSION
# ... update CHANGELOG ...
git commit -m "chore: bump VERSION to 0.5.0 for v0.5.0-jazzy"
git tag -a v0.5.0-jazzy -m "Release v0.5.0"
git push origin jazzy v0.5.0-jazzy
gh release create v0.5.0-jazzy --target jazzy
```

### Example 2: Patch Release for One Distro

Critical bug fixed on Humble only.

```bash
# Fix bug on humble
git checkout humble
git checkout -b fix/critical-memory-leak
# ... fix bug ...
git commit -m "fix: memory leak in publisher cleanup"
git push origin fix/critical-memory-leak
gh pr create --base humble --fill
# ... PR merged ...

# Release patch version
git checkout humble
git pull origin humble
echo "0.5.1" > VERSION
# ... update CHANGELOG ...
git commit -m "chore: bump VERSION to 0.5.1 for v0.5.1-humble"
git tag -a v0.5.1-humble -m "Release v0.5.1 - Critical bugfix"
git push origin humble v0.5.1-humble
gh release create v0.5.1-humble --target humble

# Jazzy stays at v0.5.0-jazzy (bug doesn't affect it)
```

### Example 3: Major Release with Breaking Changes

Releasing v1.0.0 with API changes.

```bash
# On humble (primary distro)
git checkout humble
echo "1.0.0" > VERSION

# Update CHANGELOG with breaking changes section
cat >> CHANGELOG.md <<EOF
## v1.0.0 - 2025-01-15

### ⚠️  BREAKING CHANGES

- \`Logger.Log()\` signature changed to include context
- \`Param.Get()\` now returns error instead of panic
- Removed deprecated \`LegacyInit()\` function

### Migration Guide

1. Update Logger calls:
   \`\`\`go
   // Old:
   logger.Log(severity, "message")

   // New:
   logger.Log(ctx, severity, "message")
   \`\`\`

2. Handle Param errors:
   \`\`\`go
   // Old:
   val := params.Get("key")

   // New:
   val, err := params.Get("key")
   if err != nil {
       // handle error
   }
   \`\`\`

### Features
- ...

### Bug Fixes
- ...
EOF

git add VERSION CHANGELOG.md
git commit -m "chore: bump VERSION to 1.0.0 for v1.0.0-humble"
git tag -a v1.0.0-humble -m "Release v1.0.0 - Major release with breaking changes"
git push origin humble v1.0.0-humble
gh release create v1.0.0-humble --target humble

# Repeat for jazzy if APIs are synced
```

## Hotfix Releases

### When to Do a Hotfix

- Critical security vulnerability
- Major bug affecting production use
- Data corruption or loss issue

### Hotfix Process

```bash
# 1. Create hotfix branch from release tag
git checkout -b fix/critical-security-issue v0.5.0-humble

# 2. Fix the issue
# ... make fix ...
git commit -m "fix: critical security vulnerability in param parser"

# 3. Test thoroughly
source /opt/ros/humble/setup.bash
go test ./...

# 4. PR to distro branch
git push origin fix/critical-security-issue
gh pr create --base humble --label "critical" --fill

# 5. After merge, release patch version
git checkout humble
git pull origin humble
echo "0.5.1" > VERSION
# ... update CHANGELOG with CVE details ...
git commit -m "chore: bump VERSION to 0.5.1 for v0.5.1-humble (security fix)"
git tag -a v0.5.1-humble -m "Security hotfix: CVE-XXXX-XXXX"
git push origin humble v0.5.1-humble
gh release create v0.5.1-humble --target humble

# 6. Announce security fix widely
```

## Release Checklist

Before releasing, ensure:

- [ ] All CI checks pass
- [ ] Tests pass locally on target distro
- [ ] Examples build and run
- [ ] Documentation is up to date
- [ ] CHANGELOG.md has entry for this version
- [ ] VERSION file updated
- [ ] No known critical bugs
- [ ] Breaking changes clearly documented (if major release)
- [ ] Migration guide provided (if major release)
- [ ] Tag format correct: `vX.Y.Z-distro`
- [ ] GitHub release created with release notes
- [ ] Announcement prepared (if significant release)

## Post-Release Tasks

- [ ] Update master branch to point to latest release (optional)
- [ ] Announce release on GitHub Discussions
- [ ] Update README.md installation instructions
- [ ] Close milestone (if using GitHub milestones)
- [ ] Start planning next release

## Version History Tracking

All releases should be documented in `CHANGELOG.md` and tagged in git. Users can view all releases:

```bash
# List all tags
git tag -l

# List humble releases only
git tag -l "v*-humble"

# List jazzy releases only
git tag -l "v*-jazzy"

# View a specific release
git show v0.5.0-humble
```

## FAQ

### Q: Should version numbers stay in sync across distros?

**A**: It depends:
- **In sync**: If features are kept in sync via regular porting
- **Independent**: If distros diverge significantly or get distro-specific patches

Start in sync, allow divergence when necessary.

### Q: What if I need to release different features on different distros?

**A**: Use independent versioning:
- Humble gets v0.6.0 with new feature
- Jazzy stays at v0.5.2 (feature not applicable)

### Q: How do I know what version to release?

**A**: Follow semantic versioning:
- **PATCH** (0.5.0 → 0.5.1): Bug fixes only
- **MINOR** (0.5.1 → 0.6.0): New features, backward compatible
- **MAJOR** (0.6.0 → 1.0.0): Breaking API changes

### Q: Can I delete old tags?

**A**: **No!** Tags are immutable references. Once published, never delete or change them. This ensures reproducibility and trust.

### Q: What about pre-releases?

**A**: Use pre-release tags for testing:
- `v0.5.0-rc1-humble` (release candidate)
- `v0.5.0-beta1-humble` (beta)
- `v0.5.0-alpha1-humble` (alpha)

Mark as "pre-release" in GitHub releases.

---

**Next**: See [CONTRIBUTING.md](../CONTRIBUTING.md) for development workflow.
