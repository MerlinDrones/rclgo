#!/bin/bash
# local-ci.sh: Run all CI checks locally before pushing
# This mirrors what CI does, but runs on your machine

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

error() {
    echo -e "${RED}❌ $*${NC}" >&2
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

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    error "ROS environment not sourced. Run: source /opt/ros/humble/setup.bash"
fi

info "Running local CI checks for ROS $ROS_DISTRO"
echo ""

# Track failures
FAILED_CHECKS=()

# 1. Formatting check
info "1️⃣  Checking code formatting (go fmt)..."
if ! gofmt -l . | grep -q .; then
    success "Code formatting OK"
else
    warning "Code needs formatting. Run: go fmt ./..."
    FAILED_CHECKS+=("formatting")
fi
echo ""

# 2. Go vet
info "2️⃣  Running go vet..."
if go vet ./...; then
    success "go vet passed"
else
    error "go vet failed"
    FAILED_CHECKS+=("go-vet")
fi
echo ""

# 3. Build
info "3️⃣  Building all packages..."
if go build ./...; then
    success "Build successful"
else
    error "Build failed"
    FAILED_CHECKS+=("build")
fi
echo ""

# 4. Tests
info "4️⃣  Running tests..."
if go test ./... -timeout 5m; then
    success "All tests passed"
else
    error "Tests failed"
    FAILED_CHECKS+=("tests")
fi
echo ""

# 5. Linting (if golangci-lint is installed)
info "5️⃣  Running linter..."
if command -v golangci-lint &> /dev/null; then
    if golangci-lint run --timeout=5m; then
        success "Linting passed"
    else
        warning "Linting failed (may need to fix)"
        FAILED_CHECKS+=("lint")
    fi
else
    warning "golangci-lint not installed. Install: https://golangci-lint.run/usage/install/"
    echo "   Or run: curl -sSfL https://raw.githubusercontent.com/golangci/golangci-lint/master/install.sh | sh -s -- -b \$(go env GOPATH)/bin"
fi
echo ""

# 6. Build examples
info "6️⃣  Building examples..."
EXAMPLES_FAILED=0
for example_dir in examples/*/; do
    if [ -f "${example_dir}go.mod" ]; then
        example_name=$(basename "$example_dir")
        echo "  Building example: $example_name"
        if (cd "$example_dir" && go build ./...); then
            echo "    ✓ $example_name OK"
        else
            echo "    ✗ $example_name FAILED"
            EXAMPLES_FAILED=1
        fi
    fi
done

if [ $EXAMPLES_FAILED -eq 0 ]; then
    success "All examples built successfully"
else
    warning "Some examples failed to build"
    FAILED_CHECKS+=("examples")
fi
echo ""

# Summary
echo "========================================"
echo "Summary"
echo "========================================"
echo ""

if [ ${#FAILED_CHECKS[@]} -eq 0 ]; then
    success "All checks passed! ✨"
    success "You're ready to push and create a PR"
    exit 0
else
    error "Failed checks: ${FAILED_CHECKS[*]}"
    echo ""
    echo "Fix the issues above before pushing."
    echo "Or use 'git push --no-verify' to skip pre-push checks (not recommended)."
    exit 1
fi
