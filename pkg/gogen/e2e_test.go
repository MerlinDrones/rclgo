/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// End to end: a ROS package whose interfaces stress the generator
// (testdata/rclgo_test_interfaces: constants of every type and extreme
// values, Go keywords as field names, defaults, bounded types, nested and
// external types, a service and an action) is built with colcon, its Go
// bindings are generated, and a program using them (testdata/edge_check)
// checks constants, defaults and round trips, and rclpy reads a message
// serialized by it. Skipped with -short or without colcon.
func TestGeneratedBindingsForCustomPackage(t *testing.T) {
	if testing.Short() {
		t.Skip("builds a ROS package with colcon")
	}
	distro := os.Getenv("ROS_DISTRO")
	if distro == "" {
		t.Skip("needs a sourced ROS 2 installation")
	}
	if _, err := exec.LookPath("colcon"); err != nil {
		t.Skip("colcon not found")
	}
	ws := t.TempDir()
	run := func(dir, script string) string {
		t.Helper()
		cmd := exec.Command("bash", "-c", "source /opt/ros/"+distro+"/setup.bash && "+script)
		cmd.Dir = dir
		cmd.Env = append(os.Environ(), "GOWORK=off", "PYTHONUNBUFFERED=1")
		out, err := cmd.CombinedOutput()
		if err != nil {
			t.Fatalf("%s: %v\n%s", script, err, out)
		}
		return string(out)
	}

	src := filepath.Join(ws, "src", "rclgo_test_interfaces")
	run(".", "mkdir -p "+filepath.Join(ws, "src")+" && cp -r testdata/rclgo_test_interfaces "+src)
	run(ws, "colcon build --event-handlers console_direct- >colcon.log 2>&1 || (cat colcon.log; exit 1)")
	install := filepath.Join(ws, "install", "rclgo_test_interfaces")

	mod := filepath.Join(ws, "mod")
	config := DefaultConfig
	config.RootPaths = []string{install, "/opt/ros/" + distro}
	config.DestPath = filepath.Join(mod, "msgs")
	config.CGOFlagsPath = filepath.Join(mod, "cgo-flags.env")
	config.MessageModulePrefix = "example.com/edge/msgs"
	config.RclgoImportPath = "github.com/merlindrones/rclgo"
	config.ROSPkgIncludes = []string{"rclgo_test_interfaces"}
	gen := New(&config)
	if err := gen.GenerateGolangMessageTypes(); err != nil {
		t.Fatalf("GenerateGolangMessageTypes: %v", err)
	}
	if err := gen.GenerateROS2AllMessagesImporter(); err != nil {
		t.Fatalf("GenerateROS2AllMessagesImporter: %v", err)
	}
	if err := gen.GenerateCGOFlags(); err != nil {
		t.Fatalf("GenerateCGOFlags: %v", err)
	}

	repo, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	goMod := "module example.com/edge\n\ngo 1.26.1\n\nrequire github.com/merlindrones/rclgo v0.0.0\n\nreplace github.com/merlindrones/rclgo => " + repo + "\n"
	if err := os.WriteFile(filepath.Join(mod, "go.mod"), []byte(goMod), 0o644); err != nil {
		t.Fatal(err)
	}
	run(".", "mkdir -p "+filepath.Join(mod, "check")+" && cp testdata/edge_check/main.go "+filepath.Join(mod, "check"))
	env := "source " + filepath.Join(ws, "install", "setup.bash") + " && source cgo-flags.env && "
	run(mod, env+"go mod tidy >/dev/null")
	out := run(mod, env+"go run ./check/ 2>&1")
	if !strings.Contains(out, "OK") {
		t.Fatalf("edge_check:\n%s", out)
	}
	hexLine := ""
	for _, line := range strings.Split(out, "\n") {
		if strings.HasPrefix(line, "HEX:") {
			hexLine = strings.TrimPrefix(line, "HEX:")
		}
	}
	if hexLine == "" {
		t.Fatalf("edge_check printed no message:\n%s", out)
	}

	if _, err := exec.Command("bash", "-c", "source /opt/ros/"+distro+"/setup.bash && python3 -P -c 'import rclpy'").CombinedOutput(); err != nil {
		t.Log("rclpy not available; skipping the rclpy check")
		return
	}
	script, _ := filepath.Abs("testdata/edge_check.py")
	py := run(mod, env+"python3 -P "+script+" "+hexLine)
	if strings.TrimSpace(py) != "OK" {
		t.Fatalf("rclpy read the message differently: %s", py)
	}
}
