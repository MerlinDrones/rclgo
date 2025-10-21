## Description

<!-- Provide a brief description of the changes in this PR -->

## Type of Change

<!-- Mark the relevant option with an 'x' -->

- [ ] Bug fix (non-breaking change which fixes an issue)
- [ ] New feature (non-breaking change which adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] Documentation update
- [ ] Code refactoring
- [ ] Performance improvement
- [ ] Dependency update
- [ ] Cross-distro sync (porting from another distro branch)

## Target Distro

<!-- Which ROS 2 distro is this PR targeting? -->

- [ ] Humble
- [ ] Jazzy
- [ ] Multiple distros (specify below)

**If multiple distros:** <!-- Explain how this affects each distro -->

## Cross-Distro Considerations

<!-- Answer these questions about distro compatibility -->

- [ ] This change applies to all distros and needs syncing
- [ ] This change is distro-specific (e.g., CGO bindings, API differences)
- [ ] I have tested (or plan to test) on all affected distros
- [ ] Sync PR to other distro(s) is: <!-- N/A, Planned, In Progress, or link to PR -->

## Changes Made

<!-- Describe what changed and why. List key files/functions modified -->

-
-
-

## Testing

<!-- Describe how you tested this change -->

### Manual Testing

- [ ] Built successfully (`go build ./...`)
- [ ] Tests pass (`go test ./...`)
- [ ] Examples compile and run
- [ ] Tested on target ROS 2 distro

**Test environment:**
- OS: <!-- e.g., Ubuntu 22.04 -->
- ROS 2 Distro: <!-- humble, jazzy, etc. -->
- Go Version: <!-- e.g., 1.24 -->

### Test Output

```bash
# Paste relevant test output or build logs
```

## Checklist

<!-- Mark completed items with an 'x' -->

- [ ] My code follows the coding style of this project (ran `go fmt`)
- [ ] I have run `go vet` and `golangci-lint` with no issues
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to documentation (if applicable)
- [ ] My commits follow the conventional commits format (`feat:`, `fix:`, etc.)
- [ ] I have updated ROADMAP.md if this affects feature parity tracking

## Breaking Changes

<!-- If this is a breaking change, describe the impact and migration path -->

- [ ] N/A - No breaking changes
- [ ] Breaking change details:

## Related Issues

<!-- Link any related issues, e.g., "Closes #123" or "Related to #456" -->

Closes #
Related to #

## Additional Notes

<!-- Any additional context, screenshots, or information for reviewers -->

---

## For Maintainers

<!-- Maintainer checklist - ignore if you're not a maintainer -->

- [ ] PR title follows conventional commits format
- [ ] Appropriate labels added (`distro:humble`, `distro:jazzy`, `needs-sync`, `distro-specific`, etc.)
- [ ] CI checks pass on all target distros
- [ ] Code review completed
- [ ] Merge strategy selected (squash, merge commit, or rebase)
- [ ] If merging to primary distro (humble), sync PR created for other distros
