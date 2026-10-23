#!/bin/bash
# shellcheck disable=SC2034  # config variables are read by the hooks that source this file
# SwarmOS shared hook library
#
# Sourced by every SwarmOS git hook. Canonical copy lives in
# swarmos-devtools/hooks/; do not edit the copy in a repo's .githooks/.

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

HOOKS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git rev-parse --show-toplevel)"
GIT_DIR="$(git rev-parse --git-dir)"

# Defaults; a repo can override these in .githooks/config.sh
PROTECTED_BRANCHES="master develop"
PR_BRANCH_PREFIXES="feature/ bugfix/ hotfix/"
PUSH_SCOPE="full"        # full = every module on push; changed = changed modules only
PUSH_TEST_CMD=""         # replaces "go test ./..." on push when set (e.g. "make test")
GO_MODULES=""            # modules checked on a full push; default = every tracked go.mod

if [ -f "$HOOKS_DIR/config.sh" ]; then
    # shellcheck source=/dev/null
    source "$HOOKS_DIR/config.sh"
fi

info() { echo -e "${BLUE}$*${NC}"; }
ok()   { echo -e "${GREEN}$*${NC}"; }
warn() { echo -e "${YELLOW}$*${NC}"; }
fail() { echo -e "${RED}$*${NC}"; exit 1; }

is_protected_branch() {
    local branch=$1 b
    for b in $PROTECTED_BRANCHES; do
        [ "$branch" = "$b" ] && return 0
    done
    return 1
}

in_merge() {
    [ -f "$GIT_DIR/MERGE_HEAD" ]
}

# Load ROS 2, the swarmos-nodes workspace overlay and CGO flags (bash versions).
# Harmless for repos that don't use ROS.
load_ros_env() {
    [ -n "$SWARMOS_HOOK_ENV_LOADED" ] && return 0
    export SWARMOS_HOOK_ENV_LOADED=1

    local root="${SWARMOS_ROOT:-$HOME/Git/merlin/Swarmos}"
    local ws="${ROS_WS:-$root/swarmos-nodes}"
    local distro="${ROS_DISTRO:-}"

    if [ -z "$distro" ]; then
        for d in jazzy humble; do
            [ -f "/opt/ros/$d/setup.bash" ] && distro=$d && break
        done
    fi
    # ROS setup scripts reference unset variables; don't let that abort a hook
    if [ -n "$distro" ] && [ -f "/opt/ros/$distro/setup.bash" ]; then
        # shellcheck source=/dev/null
        source "/opt/ros/$distro/setup.bash" >/dev/null 2>&1 || true
    fi
    if [ -f "$ws/install/setup.bash" ]; then
        # shellcheck source=/dev/null
        source "$ws/install/setup.bash" >/dev/null 2>&1 || true
    fi
    if [ -f "$ws/cgo-flags.env" ]; then
        # shellcheck source=/dev/null
        source "$ws/cgo-flags.env" >/dev/null 2>&1 || true
    fi
}

# Files that gofmt/vet must ignore: generated code and vendored trees
is_generated_go() {
    case "$1" in
        *.gen.go|*/vendor/*|vendor/*|*/third_party/*|third_party/*|mavlink/dialects/*|*/node_modules/*) return 0 ;;
    esac
    return 1
}

# Print the repo-relative directory of the nearest go.mod above a file ("." for root)
module_of() {
    local dir
    dir="$(dirname "$1")"
    while :; do
        [ -f "$REPO_ROOT/$dir/go.mod" ] && { echo "$dir"; return 0; }
        [ "$dir" = "." ] && return 1
        dir="$(dirname "$dir")"
    done
}

# Every tracked Go module in the repo (repo-relative dirs), unless GO_MODULES is set
all_modules() {
    if [ -n "$GO_MODULES" ]; then
        echo "$GO_MODULES" | tr ' ' '\n'
        return
    fi
    git -C "$REPO_ROOT" ls-files -- 'go.mod' '*/go.mod' | xargs -r -n1 dirname | sort -u
}

# Run go vet + go test on a module.
#   $1 = module dir (repo-relative), remaining args = package patterns (default ./...)
go_vet_and_test() {
    local mod=$1; shift
    local pkgs=("$@")
    [ ${#pkgs[@]} -eq 0 ] && pkgs=("./...")

    info "  → [$mod] go vet ${pkgs[*]}"
    (cd "$REPO_ROOT/$mod" && go vet "${pkgs[@]}") || fail "❌ go vet failed in $mod"

    info "  → [$mod] go test ${pkgs[*]}"
    (cd "$REPO_ROOT/$mod" && go test "${pkgs[@]}") || fail "❌ go test failed in $mod"
}

# Run vet + test on the packages touched by a list of changed files (one per line on stdin).
# A changed go.mod/go.sum tests the whole module.
go_checks_for_changed_files() {
    declare -A mod_pkgs=()
    declare -A mod_all=()
    local f mod rel

    while IFS= read -r f; do
        [ -z "$f" ] && continue
        case "$f" in
            *.go)
                is_generated_go "$f" && continue
                [ -d "$REPO_ROOT/$(dirname "$f")" ] || continue
                mod="$(module_of "$f")" || continue
                if [ "$mod" = "." ]; then rel="./$(dirname "$f")"; else rel="./${f#"$mod"/}"; rel="$(dirname "$rel")"; fi
                case " ${mod_pkgs[$mod]} " in *" $rel "*) ;; *) mod_pkgs[$mod]="${mod_pkgs[$mod]} $rel" ;; esac
                ;;
            go.mod|go.sum|*/go.mod|*/go.sum)
                mod="$(dirname "$f")"
                [ -f "$REPO_ROOT/$mod/go.mod" ] && mod_all[$mod]=1
                ;;
        esac
    done

    for mod in "${!mod_all[@]}"; do
        go_vet_and_test "$mod"
        unset "mod_pkgs[$mod]"
    done
    for mod in "${!mod_pkgs[@]}"; do
        # shellcheck disable=SC2086
        go_vet_and_test "$mod" ${mod_pkgs[$mod]}
    done
}

# Run .githooks/<hook>.local if the repo has one
run_local_hook() {
    local hook=$1; shift
    if [ -x "$HOOKS_DIR/$hook.local" ]; then
        "$HOOKS_DIR/$hook.local" "$@" || exit 1
    fi
}
