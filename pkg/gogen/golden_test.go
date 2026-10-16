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
	root := "/opt/ros/" + os.Getenv("ROS_DISTRO")
	if _, err := os.Stat(filepath.Join(root, "share", "test_msgs")); os.Getenv("ROS_DISTRO") == "" || err != nil {
		t.Skip("needs a sourced ROS 2 installation with test_msgs")
	}
	config := DefaultConfig
	config.RootPaths = []string{root}
	config.DestPath = dest
	config.MessageModulePrefix = "github.com/merlindrones/rclgo/pkg/msgs"
	config.RclgoImportPath = "github.com/merlindrones/rclgo"
	config.ROSPkgIncludes = goldenPackages
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
	files := map[string][]byte{}
	for _, pkg := range goldenPackages {
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
