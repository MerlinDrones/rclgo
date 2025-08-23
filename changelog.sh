#!/usr/bin/env sh
set -eu

usage() {
  cat <<EOF
Usage:
  scripts/changelog.sh [--since-tag <tag>] [--release-version <ver>] [--date YYYY-MM-DD]
                       [--in-place <CHANGELOG.md>]

Defaults:
  - If --release-version is not given, it defaults to the latest tag (if any),
    otherwise "Unreleased".
  - If release-version is a tag and --since-tag is not given, range is:
      <previous tag> .. <that tag>
    (for first tag, it includes all commits reachable from that tag)
  - If release-version is not a tag (or "Unreleased"), range is:
      <latest tag> .. HEAD  (or all commits if no tags)

Options:
  --since-tag <tag>       Start from <tag> (exclusive). Verifies tag exists.
  --release-version <ver> Section title; if a tag name, we diff up to that tag.
  --date YYYY-MM-DD       Release date (default: today; only used with a release).
  --in-place <file>       Insert the generated section at the top of <file>.
EOF
}

since_tag=""
release_version=""
release_date="$(date +%F)"
in_place_file=""

while [ $# -gt 0 ]; do
  case "$1" in
    --since-tag)        since_tag="${2:-}"; shift 2 ;;
    --release-version)  release_version="${2:-}"; shift 2 ;;
    --date)             release_date="${2:-}"; shift 2 ;;
    --in-place)         in_place_file="${2:-}"; shift 2 ;;
    -h|--help)          usage; exit 0 ;;
    *)                  echo "Unknown arg: $1" >&2; usage; exit 2 ;;
  esac
done

# Helper: does this tag exist?
tag_exists() {
  git rev-parse -q --verify "refs/tags/$1" >/dev/null 2>&1
}

# Discover tags
latest_tag="$(git describe --tags --abbrev=0 2>/dev/null || echo "")"

# Default release_version => latest tag (or Unreleased if none)
if [ -z "$release_version" ]; then
  if [ -n "$latest_tag" ]; then
    release_version="$latest_tag"
  else
    release_version="Unreleased"
  fi
fi

# Verify --since-tag if provided
if [ -n "$since_tag" ] && ! tag_exists "$since_tag"; then
  echo "Error: tag '$since_tag' does not exist." >&2
  exit 1
fi

# Determine the "release ref" we log up to:
# - If release_version is a real tag, we log up to that tag.
# - Otherwise (e.g., "Unreleased" or custom), we log up to HEAD.
if tag_exists "$release_version"; then
  release_ref="$release_version"
else
  release_ref="HEAD"
fi

# Determine the starting point:
# Priority: explicit --since-tag; else previous tag (when releasing to a tag);
# else latest tag (when releasing to HEAD); else empty (all history).
if [ -n "$since_tag" ]; then
  start_ref="$since_tag"
else
  if [ "$release_ref" != "HEAD" ]; then
    # previous tag before release_ref
    prev_tag="$(git describe --tags --abbrev=0 "${release_ref}^" 2>/dev/null || echo "")"
    start_ref="$prev_tag"
  else
    start_ref="$latest_tag"
  fi
fi

# Build git log range
if [ "$release_ref" = "HEAD" ]; then
  if [ -n "$start_ref" ]; then
    range="${start_ref}..HEAD"
  else
    range=""
  fi
else
  if [ -n "$start_ref" ]; then
    range="${start_ref}..${release_ref}"
  else
    # first tagged release: include all commits reachable from that tag
    range="${release_ref}"
  fi
fi

# Bucket temp files
tmpdir="$(mktemp -d)"
cleanup() { rm -rf "$tmpdir"; }
trap cleanup EXIT INT TERM

buckets="breaking feat fix perf refactor docs test build ci style chore revert"
for b in $buckets; do : >"$tmpdir/$b"; done

# Collect subjects
if [ -n "$range" ]; then
  git log --pretty=format:'%s' $range
else
  git log --pretty=format:'%s'
fi |
awk '
  function lcase(s){return tolower(s)}
  {
    line=$0
    if (match(line, /^([A-Za-z]+)(\([^)]+\))?(!)?:[[:space:]]*(.+)$/, m)) {
      type=lcase(m[1]); scope=m[2]; bang=m[3]; msg=m[4]
      gsub(/^\(/,"",scope); gsub(/\)$/,"",scope)
      display=(scope!="" ? "**"scope"**: "msg : msg)
      if (bang!="") print "breaking\t" display
      if (type=="feat"||type=="fix"||type=="docs"||type=="perf"||type=="refactor"||type=="test"||type=="build"||type=="ci"||type=="style"||type=="chore"||type=="revert")
        print type "\t" display
      else
        print "chore\t" display
    } else {
      print "chore\t" line
    }
  }
' |
while IFS='	' read -r bucket text; do
  [ -z "$bucket" ] && continue
  printf -- "- %s\n" "$text" >>"$tmpdir/$bucket"
done

# Build section
section="$tmpdir/section.md"
if [ "$release_ref" = "HEAD" ] && [ "$release_version" = "Unreleased" ]; then
  printf '## [Unreleased]\n' >"$section"
else
  printf '## [%s] – %s\n' "$release_version" "$release_date" >"$section"
fi

printed_any=0
print_bucket() {
  f="$1"; title="$2"
  if [ -s "$f" ]; then
    printf '\n### %s\n' "$title" >>"$section"
    cat "$f" >>"$section"
    printed_any=1
  fi
}

print_bucket "$tmpdir/breaking" "⚠️ Breaking Changes"
print_bucket "$tmpdir/feat"     "Features"
print_bucket "$tmpdir/fix"      "Bug Fixes"
print_bucket "$tmpdir/perf"     "Performance"
print_bucket "$tmpdir/refactor" "Refactoring"
print_bucket "$tmpdir/docs"     "Documentation"
print_bucket "$tmpdir/test"     "Tests"
print_bucket "$tmpdir/build"    "Build"
print_bucket "$tmpdir/ci"       "CI"
print_bucket "$tmpdir/style"    "Style"
print_bucket "$tmpdir/chore"    "Chores"
print_bucket "$tmpdir/revert"   "Reverts"

[ "$printed_any" -eq 0 ] && printf '\n(no user-facing changes)\n' >>"$section"

# Output or insert
if [ -n "$in_place_file" ]; then
  tmpout="${in_place_file}.tmp.$$"
  if [ -f "$in_place_file" ]; then
    if head -n1 "$in_place_file" | grep -q '^# Changelog'; then
      { head -n1 "$in_place_file"; echo; cat "$section"; echo; tail -n +2 "$in_place_file"; } >"$tmpout"
    else
      { echo '# Changelog'; echo; cat "$section"; echo; cat "$in_place_file"; } >"$tmpout"
    fi
  else
    { echo '# Changelog'; echo; cat "$section"; } >"$tmpout"
  fi
  mv "$tmpout" "$in_place_file"
  echo "Updated $in_place_file"
else
  cat "$section"
fi
