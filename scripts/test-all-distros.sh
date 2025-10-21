#!/bin/bash
# test-all-distros.sh: Test rclgo on all supported ROS 2 distros using Docker
# Usage: ./scripts/test-all-distros.sh [--verbose] [--distro=<distro>]

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

# Parse arguments
VERBOSE=0
SPECIFIC_DISTRO=""

for arg in "$@"; do
    case $arg in
        --verbose|-v)
            VERBOSE=1
            ;;
        --distro=*)
            SPECIFIC_DISTRO="${arg#*=}"
            ;;
        --help|-h)
            cat <<EOF
Usage: $0 [options]

Test rclgo on all supported ROS 2 distros using Docker.

Options:
  --verbose, -v          Show verbose output
  --distro=<distro>      Test only specific distro (humble, jazzy)
  --help, -h            Show this help message

Examples:
  $0                     # Test on all distros
  $0 --verbose           # Test with verbose output
  $0 --distro=humble     # Test only on Humble

EOF
            exit 0
            ;;
    esac
done

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    error "Docker is not installed or not in PATH"
fi

# Determine distros to test
if [ -n "$SPECIFIC_DISTRO" ]; then
    DISTROS=("$SPECIFIC_DISTRO")
else
    DISTROS=("humble" "jazzy")
fi

# Validate distros
for distro in "${DISTROS[@]}"; do
    case "$distro" in
        humble|jazzy) ;;
        *) error "Invalid distro: $distro" ;;
    esac
done

# Get repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$REPO_ROOT" || error "Failed to cd to repo root"

# Check for uncommitted changes
if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    warning "You have uncommitted changes. Tests will run on current working directory state."
fi

info "Testing rclgo on distros: ${DISTROS[*]}"
echo ""

# Results tracking
declare -A RESULTS

# Test each distro
for distro in "${DISTROS[@]}"; do
    info "========================================="
    info "Testing on ROS 2 $distro"
    info "========================================="
    echo ""

    # Pull latest Docker image
    info "Pulling ros:$distro Docker image..."
    if ! docker pull "ros:$distro" > /dev/null; then
        error "Failed to pull Docker image for $distro"
    fi

    # Run tests in Docker container
    info "Running tests in Docker container..."

    TEST_CMD="set -e && \
        apt-get update -qq && \
        apt-get install -y -qq \
            ros-$distro-rcl \
            ros-$distro-rcl-action \
            ros-$distro-rcl-yaml-param-parser \
            ros-$distro-test-msgs \
            build-essential > /dev/null && \
        source /opt/ros/$distro/setup.bash && \
        echo '📦 Building rclgo...' && \
        go build ./... && \
        echo '🧪 Running tests...' && \
        go test ./... -timeout 5m"

    if [ $VERBOSE -eq 1 ]; then
        TEST_CMD="${TEST_CMD//-qq/}"
        TEST_CMD="${TEST_CMD//> \/dev\/null/}"
    fi

    if docker run --rm \
        -v "$REPO_ROOT:/workspace" \
        -w /workspace \
        -e "ROS_DISTRO=$distro" \
        "ros:$distro" \
        bash -c "$TEST_CMD"; then

        RESULTS[$distro]="✅ PASS"
        success "$distro: Tests passed!"
    else
        RESULTS[$distro]="❌ FAIL"
        error "$distro: Tests failed!"
    fi

    echo ""
done

# Print summary
info "========================================="
info "Test Summary"
info "========================================="
echo ""

ALL_PASSED=1
for distro in "${DISTROS[@]}"; do
    result="${RESULTS[$distro]}"
    echo "  $distro: $result"

    if [[ "$result" == *"FAIL"* ]]; then
        ALL_PASSED=0
    fi
done

echo ""

if [ $ALL_PASSED -eq 1 ]; then
    success "All tests passed across all distros! 🎉"
    exit 0
else
    error "Some tests failed. Review the output above."
fi
