#!/bin/bash
# git-rclgo: Helper commands for rclgo multi-distro git workflow
# Usage: ./scripts/git-rclgo.sh <command> [args...]
# Or install as git alias: git config --global alias.rclgo '!bash /path/to/rclgo/scripts/git-rclgo.sh'

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Show usage
usage() {
    cat <<EOF
git-rclgo: Helper commands for rclgo multi-distro workflow

Usage:
  git-rclgo <command> [args...]

Commands:
  branch-from <distro>           Create feature branch from distro (humble, jazzy)
  sync <from> <to> [commits]     Port commits from one distro to another
  which-distro                   Show current distro context
  status                         Show git status with distro info
  test-all                       Run tests on all distros (requires Docker)
  safe-push                      Push with full checks
  quick-save [message]           Quick commit with WIP message
  oops                           Undo last commit (keep changes)

Examples:
  git-rclgo branch-from humble
  git-rclgo sync humble jazzy HEAD~3..HEAD
  git-rclgo which-distro
  git-rclgo quick-save "WIP: debugging logging"

EOF
    exit 0
}

# Command: branch-from <distro>
cmd_branch_from() {
    local distro="$1"
    if [ -z "$distro" ]; then
        error "Usage: git-rclgo branch-from <distro>"
    fi

    case "$distro" in
        humble|jazzy)
            ;;
        *)
            error "Invalid distro: $distro. Use 'humble' or 'jazzy'"
            ;;
    esac

    # Prompt for branch name
    echo "Creating feature branch from '$distro'"
    read -p "Feature name (e.g., 'logging-api'): " feature_name

    if [ -z "$feature_name" ]; then
        error "Feature name cannot be empty"
    fi

    # Prompt for branch type
    echo "Branch type:"
    echo "  1) feature/$feature_name"
    echo "  2) fix/$feature_name"
    echo "  3) experiment/$feature_name"
    read -p "Select type (1-3): " branch_type

    case "$branch_type" in
        1) prefix="feature" ;;
        2) prefix="fix" ;;
        3) prefix="experiment" ;;
        *) error "Invalid selection" ;;
    esac

    branch_name="$prefix/$feature_name"

    info "Updating $distro from remote..."
    git fetch origin "$distro"

    info "Creating branch: $branch_name"
    git checkout -b "$branch_name" "origin/$distro"

    success "Created and checked out $branch_name from $distro"
    info "Don't forget to make your first commit!"
}

# Command: sync <from> <to> [commits]
cmd_sync() {
    local from_distro="$1"
    local to_distro="$2"
    local commits="${3:-}"

    if [ -z "$from_distro" ] || [ -z "$to_distro" ]; then
        error "Usage: git-rclgo sync <from-distro> <to-distro> [commit-range]"
    fi

    # Validate distros
    for distro in "$from_distro" "$to_distro"; do
        case "$distro" in
            humble|jazzy) ;;
            *) error "Invalid distro: $distro" ;;
        esac
    done

    if [ "$from_distro" = "$to_distro" ]; then
        error "Source and target distros cannot be the same"
    fi

    info "Syncing from $from_distro to $to_distro..."

    # Update both distros
    git fetch origin "$from_distro" "$to_distro"

    # Create sync branch
    sync_branch="sync/$from_distro-to-$to_distro-$(date +%Y%m%d-%H%M%S)"
    git checkout -b "$sync_branch" "origin/$to_distro"
    success "Created sync branch: $sync_branch"

    # Cherry-pick commits if specified
    if [ -n "$commits" ]; then
        info "Cherry-picking commits: $commits"
        if git cherry-pick "$commits"; then
            success "Cherry-pick successful"
        else
            warning "Cherry-pick has conflicts. Resolve them and continue."
            exit 1
        fi
    else
        warning "No commits specified. Showing recent commits from $from_distro:"
        git log "origin/$from_distro" --oneline -10
        echo ""
        read -p "Enter commit range to cherry-pick (or 'skip' to do manually): " user_commits
        if [ "$user_commits" != "skip" ] && [ -n "$user_commits" ]; then
            git cherry-pick "$user_commits"
        fi
    fi

    info "Next steps:"
    echo "  1. Test on $to_distro: source /opt/ros/$to_distro/setup.bash && make test"
    echo "  2. Push: git push -u origin $sync_branch"
    echo "  3. Create PR: gh pr create --base $to_distro --fill"
}

# Command: which-distro
cmd_which_distro() {
    local current_branch=$(git branch --show-current)

    echo "Current branch: $current_branch"

    # Try to determine distro context
    case "$current_branch" in
        humble)
            info "Working on: humble (primary distro)"
            ;;
        jazzy)
            info "Working on: jazzy distro"
            ;;
        feature/*|fix/*|experiment/*)
            # Check which distro this branched from
            local merge_base_humble=$(git merge-base HEAD origin/humble 2>/dev/null || echo "")
            local merge_base_jazzy=$(git merge-base HEAD origin/jazzy 2>/dev/null || echo "")

            if [ -n "$merge_base_humble" ]; then
                local humble_distance=$(git rev-list --count HEAD ^origin/humble)
                info "Based on: humble (${humble_distance} commits ahead)"
            fi
            if [ -n "$merge_base_jazzy" ]; then
                local jazzy_distance=$(git rev-list --count HEAD ^origin/jazzy)
                info "Based on: jazzy (${jazzy_distance} commits ahead)"
            fi
            ;;
        sync/*)
            if [[ "$current_branch" =~ sync/(humble|jazzy)-to-(humble|jazzy) ]]; then
                local from="${BASH_REMATCH[1]}"
                local to="${BASH_REMATCH[2]}"
                info "Syncing: $from → $to"
            fi
            ;;
        *)
            warning "Unknown branch context"
            ;;
    esac
}

# Command: status
cmd_status() {
    cmd_which_distro
    echo ""
    git status
}

# Command: test-all
cmd_test_all() {
    info "Running tests on all distros..."
    "$SCRIPT_DIR/test-all-distros.sh"
}

# Command: safe-push
cmd_safe_push() {
    info "Running pre-push checks..."

    # Run tests
    if ! go test ./... -timeout 5m; then
        error "Tests failed! Fix them before pushing."
    fi

    # Run linting
    if command -v golangci-lint &> /dev/null; then
        if ! golangci-lint run --timeout=5m; then
            error "Linting failed! Fix issues before pushing."
        fi
    else
        warning "golangci-lint not found, skipping lint checks"
    fi

    success "All checks passed!"

    # Ask for confirmation
    read -p "Push to remote? (yes/no): " confirm
    if [ "$confirm" = "yes" ]; then
        git push "$@"
        success "Pushed successfully!"
    else
        info "Push cancelled"
    fi
}

# Command: quick-save
cmd_quick_save() {
    local message="${1:-WIP: quick save $(date +%Y-%m-%d_%H:%M:%S)}"

    git add -A
    git commit -m "$message"
    success "Quick save committed: $message"
}

# Command: oops
cmd_oops() {
    warning "Undoing last commit (keeping changes)..."
    git reset --soft HEAD~1
    success "Last commit undone. Changes are still staged."
    info "Use 'git restore --staged <file>' to unstage files"
}

# Main command dispatcher
main() {
    cd "$REPO_ROOT" || error "Failed to cd to repo root"

    if [ $# -eq 0 ]; then
        usage
    fi

    local command="$1"
    shift

    case "$command" in
        branch-from)
            cmd_branch_from "$@"
            ;;
        sync)
            cmd_sync "$@"
            ;;
        which-distro)
            cmd_which_distro "$@"
            ;;
        status)
            cmd_status "$@"
            ;;
        test-all)
            cmd_test_all "$@"
            ;;
        safe-push)
            cmd_safe_push "$@"
            ;;
        quick-save)
            cmd_quick_save "$@"
            ;;
        oops)
            cmd_oops "$@"
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            error "Unknown command: $command\nRun 'git-rclgo help' for usage"
            ;;
    esac
}

main "$@"
