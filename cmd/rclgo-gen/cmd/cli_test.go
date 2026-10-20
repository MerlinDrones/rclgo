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
	args := []string{"build", "-o", bin}
	if os.Getenv("GOCOVERDIR") != "" {
		// tools/coverage.sh: the binary writes its coverage there.
		args = append(args, "-cover")
		if pkgs := os.Getenv("RCLGO_COVERPKG"); pkgs != "" {
			args = append(args, "-coverpkg="+pkgs)
		}
	}
	out, err := exec.Command("go", append(args, "..")...).CombinedOutput()
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
	// generate-cgo-flags finds packages in merged (include/<pkg>) and isolated
	// (<pkg>/include/<pkg>) install trees without parsing interfaces.
	install := t.TempDir()
	for _, d := range []string{"merged/include/merged_msgs", "merged/lib", "iso/iso_msgs/include/iso_msgs", "iso/iso_msgs/lib"} {
		if err := os.MkdirAll(filepath.Join(install, d), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	check("generate-cgo-flags", "jazzy", 0, list(
		"CGO_CFLAGS=", "CGO_LDFLAGS=",
		"-I"+filepath.Join(install, "merged/include/merged_msgs"),
		"-I"+filepath.Join(install, "iso/iso_msgs/include/iso_msgs"),
		"-L"+filepath.Join(install, "iso/iso_msgs/lib"),
		"-Wl,-rpath="+filepath.Join(install, "merged/lib"),
	), nil, "generate-cgo-flags", "-r", filepath.Join(install, "merged"), "-r", filepath.Join(install, "iso"), "--output", "-")
	check("package not found", "jazzy", 1, list("no_such_package: package not found"), list("Usage:"),
		gen(good, "--include-package-deps", "no_such_package")...)
	// A mistyped root path, or a config file that can't be read, is an error
	// instead of generating nothing (or ignoring the file) with status 0.
	check("missing root path", "jazzy", 1, list("root path", "no_such_root"), list("Usage:"),
		gen(filepath.Join(root, "no_such_root"))...)
	check("missing config file", "jazzy", 1, list("config file"), nil,
		append(gen(good), "--config-file", filepath.Join(root, "no_such_config.yaml"))...)
	writeFile(t, filepath.Join(root, "bad_config.yaml"), "dest-path: [unclosed\n")
	check("malformed config file", "jazzy", 1, list("config file"), nil,
		append(gen(good), "--config-file", filepath.Join(root, "bad_config.yaml"))...)

	// A package with only .idl files (which rclgo-gen can't read) is reported.
	writeFile(t, filepath.Join(root, "idl", "share", "idl_msgs", "msg", "OnlyIdl.idl"), "module idl_msgs { };\n")
	check("idl only", "jazzy", 0, list("idl_msgs", "only .idl"), nil,
		"generate", "-r", filepath.Join(root, "idl"), "-r", good, "-d", t.TempDir(), "--cgo-flags-path", "")

	// An overlay's package replaces the underlay's whole package, as with
	// colcon overlays: a message only the underlay has is not generated.
	overlay, underlay := filepath.Join(root, "overlay"), filepath.Join(root, "underlay")
	writeFile(t, filepath.Join(overlay, "share", "layered_msgs", "msg", "Kept.msg"), "int32 a\n")
	writeFile(t, filepath.Join(underlay, "share", "layered_msgs", "msg", "Kept.msg"), "string a\n")
	writeFile(t, filepath.Join(underlay, "share", "layered_msgs", "msg", "Removed.msg"), "int32 b\n")
	dest := t.TempDir()
	check("overlay", "jazzy", 0, nil, list("Error"), "generate", "-r", overlay, "-r", underlay, "-d", dest, "--cgo-flags-path", "")
	kept, err := os.ReadFile(filepath.Join(dest, "layered_msgs", "msg", "Kept.gen.go"))
	if err != nil {
		t.Fatalf("overlay: Kept not generated: %v", err)
	}
	if !strings.Contains(string(kept), "A int32") {
		t.Errorf("overlay: Kept was generated from the underlay:\n%s", kept)
	}
	if _, err := os.Stat(filepath.Join(dest, "layered_msgs", "msg", "Removed.gen.go")); err == nil {
		t.Error("overlay: a message only the underlay has was generated")
	}

	// An absolute --dest-path inside the working directory's module gives the
	// import path of that directory as the message module prefix.
	mod := t.TempDir()
	writeFile(t, filepath.Join(mod, "go.mod"), "module example.com/m\n\ngo 1.22\n")
	writeFile(t, filepath.Join(mod, "main.go"), "package main\n\nfunc main() {}\n")
	cmd := exec.Command(bin, "generate", "-r", good, "-d", filepath.Join(mod, "gen", "msgs"), "--cgo-flags-path", "")
	cmd.Env = append(os.Environ(), "HOME="+t.TempDir(), "ROS_DISTRO=jazzy", "GOWORK=off")
	cmd.Dir = mod
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("absolute dest-path: %v\n%s", err, out)
	}
	index, err := os.ReadFile(filepath.Join(mod, "gen", "msgs", "msgs.gen.go"))
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(index), `"example.com/m/gen/msgs/good_msgs/msg"`) {
		t.Errorf("absolute dest-path: wrong message module prefix:\n%s", index)
	}

	// Include and library flags keep the order of the root paths, so an
	// overlay's headers win over the underlay's.
	for _, d := range []string{"zz_overlay/include/rcl", "zz_overlay/lib", "aa_underlay/include/rcl", "aa_underlay/lib"} {
		if err := os.MkdirAll(filepath.Join(install, d), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	cmd = exec.Command(bin, "generate-cgo-flags", "-r", filepath.Join(install, "zz_overlay"), "-r", filepath.Join(install, "aa_underlay"), "--output", "-")
	cmd.Env = append(os.Environ(), "HOME="+t.TempDir(), "ROS_DISTRO=jazzy")
	out, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("generate-cgo-flags: %v\n%s", err, out)
	}
	over := strings.Index(string(out), "-I"+filepath.Join(install, "zz_overlay/include/rcl"))
	under := strings.Index(string(out), "-I"+filepath.Join(install, "aa_underlay/include/rcl"))
	if over < 0 || under < 0 || over > under {
		t.Errorf("the overlay's include flag must come first:\n%s", out)
	}
}
