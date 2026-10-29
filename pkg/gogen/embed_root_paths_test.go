/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"bytes"
	"io/fs"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

func TestEmbeddedRootPaths(t *testing.T) {
	roots := []string{"/home/dev/ws/install/px4_msgs", "/opt/ros/jazzy", "/opt/ros/jazzy/", "/usr", "/opt/rosx/jazzy"}
	cases := []struct {
		name     string
		patterns []string
		want     []string
	}{
		{"default keeps distro installs and /usr", nil, []string{"/opt/ros/jazzy", "/opt/ros/jazzy/", "/usr"}},
		{"explicit patterns", []string{"/usr", "/home/*/ws/install/*"}, []string{"/home/dev/ws/install/px4_msgs", "/usr"}},
		{"empty list embeds nothing", []string{}, nil},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			got := Config{RootPaths: roots, EmbedRootPaths: c.patterns}.EmbeddedRootPaths()
			if !reflect.DeepEqual(got, c.want) {
				t.Errorf("EmbeddedRootPaths() = %q, want %q", got, c.want)
			}
		})
	}
}

// distroRoot returns the ROS 2 distribution install (/opt/ros/<distro>) or
// skips the test.
func distroRoot(t *testing.T) string {
	t.Helper()
	if d := os.Getenv("ROS_DISTRO"); d != "" {
		if root := filepath.Join("/opt/ros", d); dirExists(root) {
			return root
		}
	}
	t.Skip("no ROS 2 distribution under /opt/ros (source /opt/ros/<distro>/setup.bash)")
	return ""
}

func dirExists(p string) bool {
	fi, err := os.Stat(p)
	return err == nil && fi.IsDir()
}

// generateWithOverlay generates the primitives, the rclgo cgo flags and the
// std_msgs bindings with an extra (overlay) root path in front of the
// distribution install, and returns the generated files by relative path.
func generateWithOverlay(t *testing.T, distro, overlay string) map[string][]byte {
	t.Helper()
	dest := t.TempDir()
	rule, err := NewRule("^std_msgs$")
	if err != nil {
		t.Fatal(err)
	}
	g := New(&Config{
		RclgoImportPath:     DefaultConfig.RclgoImportPath,
		MessageModulePrefix: DefaultConfig.MessageModulePrefix,
		RootPaths:           []string{overlay, distro},
		DestPath:            dest,
		RegexIncludes:       RuleSet{rule},
	})
	for name, gen := range map[string]func() error{
		"primitives": g.GeneratePrimitives, "rclgo flags": g.GenerateRclgoFlags, "messages": g.GenerateGolangMessageTypes,
	} {
		if err := gen(); err != nil {
			t.Fatalf("generate %s: %v", name, err)
		}
	}
	files := map[string][]byte{}
	err = filepath.WalkDir(dest, func(p string, d fs.DirEntry, err error) error {
		if err != nil || d.IsDir() {
			return err
		}
		b, err := os.ReadFile(p)
		rel, _ := filepath.Rel(dest, p)
		files[rel] = b
		return err
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(files) == 0 {
		t.Fatal("nothing was generated")
	}
	return files
}

// Generated code is committed, so it must not depend on where the
// generating machine keeps its workspace overlays.
func TestGeneratedCodeDoesNotDependOnOverlayRoots(t *testing.T) {
	distro := distroRoot(t)
	overlayA := filepath.Join(t.TempDir(), "ws_a", "install")
	overlayB := filepath.Join(t.TempDir(), "other", "ws_b", "install")
	for _, d := range []string{overlayA, overlayB} {
		if err := os.MkdirAll(filepath.Join(d, "share"), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	a := generateWithOverlay(t, distro, overlayA)
	b := generateWithOverlay(t, distro, overlayB)
	if len(a) != len(b) {
		t.Fatalf("generated %d files with one overlay and %d with the other", len(a), len(b))
	}
	embedsDistro := false
	for rel, ca := range a {
		cb, ok := b[rel]
		if !ok {
			t.Errorf("%s generated with one overlay only", rel)
			continue
		}
		if !bytes.Equal(ca, cb) {
			t.Errorf("%s differs between the two overlays", rel)
		}
		for _, o := range []string{overlayA, overlayB} {
			if bytes.Contains(ca, []byte(o)) || bytes.Contains(cb, []byte(o)) {
				t.Errorf("%s embeds the overlay root %s", rel, o)
			}
		}
		if strings.Contains(string(ca), `"-I`+distro+`/include/`) {
			embedsDistro = true
		}
	}
	if !embedsDistro {
		t.Errorf("no generated file embeds the distribution install %s", distro)
	}
}
