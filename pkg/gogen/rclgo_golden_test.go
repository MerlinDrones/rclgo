/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"bytes"
	"fmt"
	"go/format"
	"os"
	"path/filepath"
	"testing"
)

// generate-rclgo regenerates code that is part of rclgo itself; the result
// must match what is committed (see the generate target in the Makefile),
// up to gofmt. It is generated twice, because the generator once kept state
// between runs in package variables.
func TestGenerateRclgoMatchesCommittedCode(t *testing.T) {
	distro := os.Getenv("ROS_DISTRO")
	root := "/opt/ros/" + distro
	if _, err := os.Stat(filepath.Join(root, "include", "rcl")); distro == "" || err != nil {
		t.Skip("needs a sourced ROS 2 installation")
	}
	for run := 1; run <= 2; run++ {
		t.Run(fmt.Sprintf("run %d", run), func(t *testing.T) { checkGeneratedRclgo(t, root) })
	}
}

func checkGeneratedRclgo(t *testing.T, root string) {
	dest := t.TempDir()
	config := DefaultConfig
	config.RootPaths = []string{"/usr", root}
	config.DestPath = dest
	gen := New(&config)
	for name, f := range map[string]func() error{
		"GeneratePrimitives":     gen.GeneratePrimitives,
		"GenerateRclgoFlags":     gen.GenerateRclgoFlags,
		"GenerateTestGogenFlags": gen.GenerateTestGogenFlags,
		"GenerateROS2ErrorTypes": gen.GenerateROS2ErrorTypes,
	} {
		if err := f(); err != nil {
			t.Fatalf("%s: %v", name, err)
		}
	}
	for _, f := range []string{
		"pkg/rclgo/primitives/primitives.gen.go",
		"pkg/rclgo/flags.gen.go",
		"tests/gogen/flags.gen.go",
		"pkg/rclgo/errortypes.gen.go",
	} {
		want, err := os.ReadFile(filepath.Join("../..", f))
		if err != nil {
			t.Fatal(err)
		}
		got, err := os.ReadFile(filepath.Join(dest, f))
		if err != nil {
			t.Fatalf("%s not generated: %v", f, err)
		}
		// Committed files are gofmt'd; compare the formatted code.
		if w, err := format.Source(want); err == nil {
			want = w
		}
		if g, err := format.Source(got); err != nil {
			t.Errorf("%s does not parse as Go: %v", f, err)
		} else {
			got = g
		}
		if !bytes.Equal(want, got) {
			t.Errorf("%s differs from the committed file", f)
		}
	}
}
