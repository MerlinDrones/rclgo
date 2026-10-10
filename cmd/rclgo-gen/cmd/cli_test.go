/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package cmd_test

import (
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// The rclgo-gen command line: help, flag errors and generation errors give
// the expected output and exit status. The binary is built once and run as a
// subprocess, so exit codes are the real ones.

func buildCLI(t *testing.T) string {
	t.Helper()
	bin := filepath.Join(t.TempDir(), "rclgo-gen")
	out, err := exec.Command("go", "build", "-o", bin, "..").CombinedOutput()
	if err != nil {
		t.Fatalf("build rclgo-gen: %v\n%s", err, out)
	}
	return bin
}

func writeFile(t *testing.T, path, content string) {
	t.Helper()
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
		t.Fatal(err)
	}
}

func TestCLI(t *testing.T) {
	bin := buildCLI(t)
	root := t.TempDir()
	good := filepath.Join(root, "good")
	bad := filepath.Join(root, "bad")
	writeFile(t, filepath.Join(good, "share", "good_msgs", "msg", "Good.msg"), "int32 a\nstring b \"x\"\n")
	writeFile(t, filepath.Join(bad, "share", "bad_msgs", "msg", "Bad.msg"), "int32 a\nthis is not valid\n")

	// check runs rclgo-gen and checks its exit status and output.
	check := func(name, distro string, wantCode int, contains, excludes []string, args ...string) {
		t.Helper()
		cmd := exec.Command(bin, args...)
		cmd.Env = append(os.Environ(), "HOME="+t.TempDir(), "ROS_DISTRO="+distro)
		cmd.Dir = t.TempDir()
		b, err := cmd.CombinedOutput()
		out, code := string(b), 0
		var exit *exec.ExitError
		if errors.As(err, &exit) {
			code = exit.ExitCode()
		} else if err != nil {
			t.Fatalf("%s: %v", name, err)
		}
		if code != wantCode {
			t.Errorf("%s: exit status %d, want %d:\n%s", name, code, wantCode, out)
		}
		for _, s := range contains {
			if !strings.Contains(out, s) {
				t.Errorf("%s: output lacks %q:\n%s", name, s, out)
			}
		}
		for _, s := range excludes {
			if strings.Contains(out, s) {
				t.Errorf("%s: output contains %q:\n%s", name, s, out)
			}
		}
		if n := strings.Count(out, "Error:"); n > 1 {
			t.Errorf("%s: the error is printed %d times:\n%s", name, n, out)
		}
	}
	gen := func(rootPath string, extra ...string) []string {
		return append([]string{"generate", "-r", rootPath, "-d", t.TempDir(), "--cgo-flags-path", ""}, extra...)
	}
	list := func(s ...string) []string { return s }

	check("help", "jazzy", 0, list("generate", "Usage:"), nil, "--help")
	check("generate help", "jazzy", 0, list("--root-path", "--include-package-deps"), nil, "generate", "--help")
	// Bad flags and arguments show the usage.
	check("unknown flag", "jazzy", 1, list("unknown flag: --no-such-flag", "Usage:"), nil, "generate", "--no-such-flag")
	check("unknown command", "jazzy", 1, list("unknown command"), nil, "no-such-command")
	check("wrong distro", "humble", 1, list(`ROS_DISTRO should be set to "jazzy"`), nil, gen(good)...)
	check("good package", "jazzy", 0, nil, list("Error"), gen(good)...)
	// Generation errors exit with status 1 and report the file, line and
	// problem once, without the usage text.
	check("bad package", "jazzy", 1, list("Bad.msg", "line 2", "unknown type 'this'"), list("Usage:"), gen(bad)...)
	check("package not found", "jazzy", 1, list("no_such_package: package not found"), list("Usage:"),
		gen(good, "--include-package-deps", "no_such_package")...)
}
