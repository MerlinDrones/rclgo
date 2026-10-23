#!/bin/bash
# Print the packages of the Go module in the current directory that can be
# vetted without ROS: neither the package, its test files, nor any of their
# non-standard-library dependencies use CGO (rclgo, gocv, ...).
#
# Managed by swarmos-devtools; do not edit in place.

set -euo pipefail

# Non-std packages with cgo files, among everything ./... (and its tests) reaches
cgo_pkgs="$(go list -e -test -deps -f '{{if and .CgoFiles (not .Standard)}}{{.ImportPath}}{{end}}' ./... \
    | sed 's/ \[.*//' | sort -u)"

# One line per package (and test variant): "<import path>|<deps>"
go list -e -test -f '{{.ImportPath}}|{{join .Deps " "}}' ./... \
    | awk -F'|' -v cgo="$cgo_pkgs" '
        BEGIN { n = split(cgo, c, "\n"); for (i = 1; i <= n; i++) if (c[i] != "") bad_dep[c[i]] = 1 }
        {
            pkg = $1
            sub(/ \[.*/, "", pkg)        # "a/b [a/b.test]" -> "a/b"
            sub(/\.test$/, "", pkg)      # "a/b.test"       -> "a/b"
            sub(/_test$/, "", pkg)       # "a/b_test"       -> "a/b" (external test package)
            seen[pkg] = 1
            if (pkg in bad_dep) bad[pkg] = 1
            m = split($2, d, " ")
            for (i = 1; i <= m; i++) {
                dep = d[i]; sub(/ \[.*/, "", dep)
                if (dep in bad_dep) { bad[pkg] = 1; break }
            }
        }
        END { for (p in seen) if (!(p in bad)) print p }' \
    | sort
