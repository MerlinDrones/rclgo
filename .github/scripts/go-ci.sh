#!/bin/bash
# SwarmOS Go checks — managed by swarmos-devtools; do not edit in place.
#
# USAGE: go-ci.sh <target-branch>
#
#   develop  → build like a developer does: clone the MerlinDrones modules this
#              repo depends on (and theirs) at their working branch, put them
#              in a go.work, and go vet. No tidy check: go mod tidy ignores
#              go.work, so it would fail on unreleased dependency code.
#   anything else (master, rclgo's jazzy/humble)
#            → build like a release: go.mod versions only (no go.work),
#              go mod tidy check + go vet.
#
# go vet only covers packages that build without ROS (go-pure-packages.sh).

set -uo pipefail

TARGET_BRANCH="${1:?usage: go-ci.sh <target-branch>}"
ROOT="$(git rev-parse --show-toplevel)"
HERE="$(cd "$(dirname "$0")" && pwd)"
SELF="$(basename "${GITHUB_REPOSITORY:-$ROOT}")"
SIBLINGS="$ROOT/.ci-siblings"

# Modules to check: GO_MODULES from the repo's hook config (same setting the
# pre-push hook uses, e.g. rclgo skips its examples), else every tracked go.mod.
GO_MODULES=""
# shellcheck source=/dev/null
[ -f "$ROOT/.githooks/config.sh" ] && source "$ROOT/.githooks/config.sh"
if [ -n "$GO_MODULES" ]; then
    read -r -a MODULES <<< "$GO_MODULES"
else
    mapfile -t MODULES < <(git -C "$ROOT" ls-files 'go.mod' '*/go.mod' | xargs -r -n1 dirname | sort -u)
fi

# Module paths provided by this repo itself (e.g. swarmos-deployment's
# staging/demo/target-injector) — never cloned as dependencies
declare -A LOCAL_PATHS=()
while read -r p; do LOCAL_PATHS[$p]=1; done < <(
    git -C "$ROOT" ls-files 'go.mod' '*/go.mod' | while read -r f; do awk '$1 == "module" {print $2; exit}' "$ROOT/$f"; done)

# Working branch of a MerlinDrones repo (rclgo is pegged to a ROS 2 distro)
working_branch() {
    case "$1" in
        rclgo) echo jazzy ;;
        *) echo develop ;;
    esac
}

# MerlinDrones repos required by a go.mod (module path's first element after
# github.com/merlindrones/), skipping modules that live in this repo
merlin_deps() {
    go mod edit -json "$1" | grep -oE '"Path": "github\.com/merlindrones/[^"]+' | sed 's/^"Path": "//' | sort -u |
        while read -r path; do
            [ -n "${LOCAL_PATHS[$path]:-}" ] && continue
            echo "$path" | cut -d/ -f3
        done | sort -u
}

status=0

if [ "$TARGET_BRANCH" = "develop" ]; then
    echo "Target $TARGET_BRANCH: go.work against MerlinDrones dependencies' working branches"

    # Collect dependency repos transitively
    declare -A seen=(["$SELF"]=1)
    queue=()
    for mod in "${MODULES[@]}"; do
        mapfile -t -O "${#queue[@]}" queue < <(merlin_deps "$ROOT/$mod/go.mod")
    done
    mkdir -p "$SIBLINGS"
    while [ ${#queue[@]} -gt 0 ]; do
        repo="${queue[0]}"; queue=("${queue[@]:1}")
        [ -n "${seen[$repo]:-}" ] && continue
        seen[$repo]=1
        branch="$(working_branch "$repo")"
        echo "  cloning $repo@$branch"
        if ! git clone -q --depth 1 -b "$branch" "https://github.com/MerlinDrones/$repo" "$SIBLINGS/$repo"; then
            echo "::error::could not clone MerlinDrones/$repo@$branch"
            exit 1
        fi
        mapfile -t -O "${#queue[@]}" queue < <(merlin_deps "$SIBLINGS/$repo/go.mod")
    done

    (
        cd "$ROOT" || exit 1
        rm -f go.work go.work.sum
        go work init
        for mod in "${MODULES[@]}"; do go work use "./$mod"; done
        for dir in "$SIBLINGS"/*/; do [ -f "$dir/go.mod" ] && go work use "${dir%/}"; done
        echo "go.work:"; sed 's/^/  /' go.work
    ) || exit 1
    export GOWORK="$ROOT/go.work"
else
    echo "Target $TARGET_BRANCH: release build — go.mod versions only"
    export GOWORK=off
fi

for mod in "${MODULES[@]}"; do
    echo "::group::$mod"
    pushd "$ROOT/$mod" >/dev/null || exit 1

    if [ "$GOWORK" = "off" ] && ! go mod tidy -diff; then
        echo "::error::$mod: go.mod/go.sum are not tidy — run 'go mod tidy' in $mod"
        status=1
    fi

    pkgs="$(bash "$HERE/go-pure-packages.sh")"
    if [ -n "$pkgs" ]; then
        # shellcheck disable=SC2086
        go vet $pkgs || { echo "::error::$mod: go vet failed"; status=1; }
    else
        echo "$mod: every package needs ROS/CGO — vet runs locally only"
    fi

    popd >/dev/null || exit 1
    echo "::endgroup::"
done

exit $status
