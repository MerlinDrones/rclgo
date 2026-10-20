package faultinject_test

import (
	"bytes"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
)

// The wrappers cover every function rclgo calls: regenerating them changes
// nothing. If this fails, run go generate in this package.
func TestWrappersUpToDate(t *testing.T) {
	if os.Getenv("ROS_DISTRO") == "" {
		rclgotest.Unavailable(t, "needs a sourced ROS 2 installation")
	}
	dir := t.TempDir()
	out, err := exec.Command("go", "run", "./gen", "-out", dir).CombinedOutput()
	if err != nil {
		t.Fatalf("go run ./gen: %v\n%s", err, out)
	}
	for _, f := range []string{"wrappers.gen.c", "wrappers.gen.go"} {
		want, err := os.ReadFile(f)
		if err != nil {
			t.Fatal(err)
		}
		got, err := os.ReadFile(filepath.Join(dir, f))
		if err != nil {
			t.Fatal(err)
		}
		if !bytes.Equal(want, got) {
			t.Errorf("%s is out of date; run go generate in pkg/rclgo/internal/faultinject", f)
		}
	}
}
