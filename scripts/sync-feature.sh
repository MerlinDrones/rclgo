#!/bin/bash
# sync-feature.sh: Semi-automated script for porting features between ROS 2 distros
# Usage: ./scripts/sync-feature.sh <from-distro> <to-distro> <commit-range>

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

error() {
    echo -e "${RED}❌ Error: $*${NC}" >&2
    exit 1
}

success() {
    echo -e "${GREEN}✅ $*${NC}"
}

warning() {
    echo -e "${YELLOW}⚠️  $*${NC}"
}

info() {
    echo -e "${BLUE}ℹ️  $*${NC}"
}

# Usage
if [ $# -lt 2 ]; then
    cat <<EOF
Usage: $0 <from-distro> <to-distro> [commit-range]

Ports commits from one ROS 2 distro branch to another.

Arguments:
  from-distro    Source distro (humble, jazzy)
  to-distro      Target distro (humble, jazzy)
  commit-range   Git commit range to cherry-pick (optional, will prompt if not provided)

Examples:
  $0 humble jazzy HEAD~5..HEAD
  $0 humble jazzy abc123..def456
  $0 humble jazzy  # Will show recent commits and prompt

EOF
    exit 1
fi

FROM_DISTRO="$1"
TO_DISTRO="$2"
COMMIT_RANGE="${3:-}"

# Validate distros
for distro in "$FROM_DISTRO" "$TO_DISTRO"; do
    case "$distro" in
        humble|jazzy) ;;
        *) error "Invalid distro: $distro. Must be 'humble' or 'jazzy'" ;;
    esac
done

if [ "$FROM_DISTRO" = "$TO_DISTRO" ]; then
    error "Source and target distros cannot be the same"
fi

# Check if we're in the rclgo repo
if [ ! -f "go.mod" ] || ! grep -q "github.com/merlindrones/rclgo" go.mod; then
    error "This script must be run from the rclgo repository root"
fi

# Check for uncommitted changes
if ! git diff-index --quiet HEAD --; then
    error "You have uncommitted changes. Please commit or stash them first."
fi

info "Syncing features: $FROM_DISTRO → $TO_DISTRO"
echo ""

# Fetch latest from remote
info "Fetching latest changes from remote..."
git fetch origin "$FROM_DISTRO" "$TO_DISTRO"
success "Fetched successfully"
echo ""

# Show recent commits from source distro
if [ -z "$COMMIT_RANGE" ]; then
    info "Recent commits on $FROM_DISTRO:"
    git log "origin/$FROM_DISTRO" --oneline --decorate --graph -20
    echo ""

    read -p "Enter commit range to sync (e.g., HEAD~5..HEAD or abc123..def456): " COMMIT_RANGE

    if [ -z "$COMMIT_RANGE" ]; then
        error "Commit range cannot be empty"
    fi
fi

# Expand commit range to full hashes
info "Resolving commits in range: $COMMIT_RANGE"
COMMITS=$(git log --reverse --pretty=format:"%H" "origin/$FROM_DISTRO" "$COMMIT_RANGE")

if [ -z "$COMMITS" ]; then
    error "No commits found in range: $COMMIT_RANGE"
fi

COMMIT_COUNT=$(echo "$COMMITS" | wc -l)
info "Found $COMMIT_COUNT commit(s) to sync"
echo ""

# Show commits to be synced
info "Commits to be synced:"
echo "$COMMITS" | while read -r commit; do
    git log --oneline --format="%h %s" "$commit" -1
done
echo ""

# Confirm with user
read -p "Proceed with syncing these commits to $TO_DISTRO? (yes/no): " confirm
if [ "$confirm" != "yes" ]; then
    info "Sync cancelled"
    exit 0
fi

# Create sync branch
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
SYNC_BRANCH="sync/$FROM_DISTRO-to-$TO_DISTRO-$TIMESTAMP"

info "Creating sync branch: $SYNC_BRANCH"
git checkout -b "$SYNC_BRANCH" "origin/$TO_DISTRO"
success "Branch created and checked out"
echo ""

# Cherry-pick commits one by one
info "Cherry-picking commits..."
FAILED=0
SUCCEEDED=0

echo "$COMMITS" | while read -r commit; do
    commit_msg=$(git log --oneline --format="%s" "$commit" -1)
    info "Picking: $commit - $commit_msg"

    if git cherry-pick "$commit"; then
        success "  ✓ Applied successfully"
        SUCCEEDED=$((SUCCEEDED + 1))
    else
        warning "  ✗ Cherry-pick failed (conflicts)"
        echo ""
        warning "Resolve conflicts, then:"
        echo "  - Fix conflicts in your editor"
        echo "  - git add <resolved-files>"
        echo "  - git cherry-pick --continue"
        echo ""
        echo "Or skip this commit:"
        echo "  - git cherry-pick --skip"
        echo ""
        echo "Or abort the sync:"
        echo "  - git cherry-pick --abort"
        echo "  - git checkout $FROM_DISTRO"
        echo "  - git branch -D $SYNC_BRANCH"

        FAILED=1
        break
    fi
done

if [ $FAILED -eq 1 ]; then
    error "Cherry-pick process halted due to conflicts. Resolve them and continue manually."
fi

echo ""
success "All commits cherry-picked successfully!"
echo ""

# Test the changes
info "Next steps:"
echo ""
echo "1️⃣  Test on $TO_DISTRO environment:"
echo "    source /opt/ros/$TO_DISTRO/setup.bash"
echo "    go build ./..."
echo "    go test ./..."
echo ""
echo "2️⃣  Review the changes:"
echo "    git log origin/$TO_DISTRO..$SYNC_BRANCH"
echo "    git diff origin/$TO_DISTRO..$SYNC_BRANCH"
echo ""
echo "3️⃣  Push to remote:"
echo "    git push -u origin $SYNC_BRANCH"
echo ""
echo "4️⃣  Create pull request:"
echo "    gh pr create --base $TO_DISTRO --title \"sync: <description> from $FROM_DISTRO\" --fill"
echo ""

# Offer to run tests automatically
read -p "Run tests now? (yes/no): " run_tests
if [ "$run_tests" = "yes" ]; then
    if [ -f "/opt/ros/$TO_DISTRO/setup.bash" ]; then
        info "Running tests on $TO_DISTRO..."
        bash -c "source /opt/ros/$TO_DISTRO/setup.bash && go test ./... -timeout 5m" || warning "Tests failed. Review and fix before creating PR."
    else
        warning "/opt/ros/$TO_DISTRO/setup.bash not found. Cannot run tests."
    fi
fi

success "Sync process complete!"
info "You are now on branch: $SYNC_BRANCH"
