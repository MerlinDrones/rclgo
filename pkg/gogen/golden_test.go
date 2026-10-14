package gogen

import (
	"bytes"
	"go/format"
	"io/fs"
	"os"
	"path/filepath"
	"testing"
)

// The generator's output for installed ROS packages must be deterministic and
// match the generated code committed under pkg/msgs, so that templates and
// committed code cannot drift apart unnoticed.

var goldenPackages = []string{"test_msgs", "example_interfaces"}

func generateInto(t *testing.T, dest string) {
	t.Helper()
	generatePackagesInto(t, dest, goldenPackages)
}

func generatePackagesInto(t *testing.T, dest string, pkgs []string) {
	t.Helper()
	root := "/opt/ros/" + os.Getenv("ROS_DISTRO")
	if _, err := os.Stat(filepath.Join(root, "share", "test_msgs")); os.Getenv("ROS_DISTRO") == "" || err != nil {
		unavailable(t, "needs a sourced ROS 2 installation with test_msgs")
	}
	config := DefaultConfig
	config.RootPaths = []string{root}
	config.DestPath = dest
	config.MessageModulePrefix = "github.com/merlindrones/rclgo/pkg/msgs"
	config.RclgoImportPath = "github.com/merlindrones/rclgo"
	config.ROSPkgIncludes = pkgs
	config.RegexIncludes = nil
	config.GoPkgIncludes = nil
	if err := New(&config).GenerateGolangMessageTypes(); err != nil {
		t.Fatalf("GenerateGolangMessageTypes: %v", err)
	}
}

// readTree returns the gofmt'd contents of every .go file under dir/pkg, keyed
// by path relative to dir.
func readTree(t *testing.T, dir string) map[string][]byte {
	t.Helper()
	return readPackages(t, dir, goldenPackages)
}

func readPackages(t *testing.T, dir string, pkgs []string) map[string][]byte {
	t.Helper()
	files := map[string][]byte{}
	for _, pkg := range pkgs {
		err := filepath.WalkDir(filepath.Join(dir, pkg), func(path string, d fs.DirEntry, err error) error {
			if err != nil || d.IsDir() || filepath.Ext(path) != ".go" {
				return err
			}
			src, err := os.ReadFile(path)
			if err != nil {
				return err
			}
			formatted, err := format.Source(src)
			if err != nil {
				t.Errorf("%s does not parse as Go: %v", path, err)
				formatted = src
			}
			rel, _ := filepath.Rel(dir, path)
			files[rel] = formatted
			return nil
		})
		if err != nil {
			t.Fatalf("walk %s: %v", pkg, err)
		}
	}
	return files
}

func compareTrees(t *testing.T, wantName string, want map[string][]byte, gotName string, got map[string][]byte) {
	t.Helper()
	for name, w := range want {
		g, ok := got[name]
		if !ok {
			t.Errorf("%s has %s, %s does not", wantName, name, gotName)
			continue
		}
		if !bytes.Equal(w, g) {
			t.Errorf("%s differs between %s and %s", name, wantName, gotName)
		}
	}
	for name := range got {
		if _, ok := want[name]; !ok {
			t.Errorf("%s has %s, %s does not", gotName, name, wantName)
		}
	}
}

func TestGeneratorOutputIsDeterministic(t *testing.T) {
	a, b := t.TempDir(), t.TempDir()
	generateInto(t, a)
	generateInto(t, b)
	compareTrees(t, "first run", readTree(t, a), "second run", readTree(t, b))
}

func TestGeneratorOutputMatchesCommittedCode(t *testing.T) {
	dest := t.TempDir()
	generateInto(t, dest)
	compareTrees(t, "pkg/msgs", readTree(t, "../msgs"), "fresh generation", readTree(t, dest))
}

// Every bundled package in pkg/msgs that the installed ROS distribution has
// regenerates to the committed code. Files only one side has are logged, not
// failed: they follow the installed distribution's version (new upstream
// messages), not the generator.
func TestAllBundledPackagesMatchCommittedCode(t *testing.T) {
	if testing.Short() {
		t.Skip("generates every bundled package")
	}
	root := "/opt/ros/" + os.Getenv("ROS_DISTRO")
	entries, err := os.ReadDir("../msgs")
	if err != nil {
		t.Fatal(err)
	}
	var pkgs []string
	for _, e := range entries {
		if !e.IsDir() {
			continue
		}
		if _, err := os.Stat(filepath.Join(root, "share", e.Name())); err == nil {
			pkgs = append(pkgs, e.Name())
		} else {
			t.Logf("%s is not installed under %s; not compared", e.Name(), root)
		}
	}
	if len(pkgs) < 2 {
		unavailable(t, "needs a sourced ROS 2 installation")
	}
	dest := t.TempDir()
	generatePackagesInto(t, dest, pkgs)
	want, got := readPackages(t, "../msgs", pkgs), readPackages(t, dest, pkgs)
	// A package (and type: msg, srv, action) whose interfaces differ between
	// the installed distribution and pkg/msgs also differs in common.gen.go,
	// which lists the C headers and libraries of its dependencies.
	versionDiffers := map[string]bool{}
	for name := range want {
		if _, ok := got[name]; !ok {
			t.Logf("pkg/msgs has %s, the installed distribution doesn't", name)
			versionDiffers[filepath.Dir(name)] = true
		}
	}
	for name := range got {
		if _, ok := want[name]; !ok {
			t.Logf("the installed distribution has %s, pkg/msgs doesn't", name)
			versionDiffers[filepath.Dir(name)] = true
		}
	}
	compared := 0
	for name, w := range want {
		g, ok := got[name]
		if !ok {
			continue
		}
		if filepath.Base(name) == "common.gen.go" && versionDiffers[filepath.Dir(name)] {
			t.Logf("%s not compared: its package's interfaces differ by distribution version", name)
			continue
		}
		compared++
		if !bytes.Equal(w, g) {
			t.Errorf("%s differs between pkg/msgs and a fresh generation", name)
		}
	}
	t.Logf("compared %d files of %d packages", compared, len(pkgs))
	if compared < 100 {
		t.Fatalf("only %d files compared", compared)
	}
}
