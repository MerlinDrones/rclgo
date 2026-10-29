/*
This file is part of rclgo

Copyright © 2023 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package test

import (
	"io/fs"
	"os"
	"os/exec"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

func skipIfShort(t *testing.T) {
	t.Helper()
	if testing.Short() {
		t.Skip("skipping test in short mode.")
	}
}

const shellPrelude = `
set -eo pipefail
function kill-jobs {
	kill $(jobs -p)
}
trap 'kill-jobs' SIGINT SIGTERM EXIT
`

// shell runs script with bash in dir.
func shell(t *testing.T, dir, script string) {
	t.Helper()
	cmd := exec.Command("bash", "-c", shellPrelude+script)
	cmd.Dir = dir
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	require.NoError(t, cmd.Run())
}

// copyExample copies the example directory src into a temporary directory,
// so that building it leaves the source tree alone, and points the go.mod
// files in modules (relative to src) at this checkout of rclgo. It returns
// the copy's path. Build output from earlier runs in the source tree (colcon's
// build, install and log, generated msgs, binaries, the pipe) is left out.
func copyExample(t *testing.T, src string, modules ...string) string {
	t.Helper()
	root, err := filepath.Abs("..")
	require.NoError(t, err)
	dst := filepath.Join(t.TempDir(), filepath.Base(src))
	err = filepath.WalkDir(src, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		rel, err := filepath.Rel(src, path)
		if err != nil {
			return err
		}
		if d.IsDir() {
			switch d.Name() {
			case "build", "install", "log", "msgs":
				return filepath.SkipDir
			}
			return os.MkdirAll(filepath.Join(dst, rel), 0o755)
		}
		info, err := d.Info()
		if err != nil {
			return err
		}
		if !info.Mode().IsRegular() || info.Mode()&0o111 != 0 {
			return nil // pipes and built binaries
		}
		data, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		return os.WriteFile(filepath.Join(dst, rel), data, info.Mode().Perm())
	})
	require.NoError(t, err)
	for _, m := range modules {
		cmd := exec.Command("go", "mod", "edit", "-replace", "github.com/merlindrones/rclgo="+root)
		cmd.Dir = filepath.Join(dst, m)
		out, err := cmd.CombinedOutput()
		require.NoError(t, err, "%s", out)
	}
	return dst
}

func TestCustomMessagePackage(t *testing.T) {
	skipIfShort(t)
	shell(t, copyExample(t, "custom_message_package", "greeter"), `
cd greeting_msgs
colcon build
. install/local_setup.sh

cd ../greeter
go generate
go build

./greeter &
msg=$(ros2 topic echo --once /greeter/hello greeting_msgs/msg/Hello)
test "$msg" == $'name: gopher\n---'
`)
}

func TestPublisherSubscriber(t *testing.T) {
	skipIfShort(t)
	shell(t, copyExample(t, "publisher_subscriber", "."), `
go generate

go build -o pub ./publisher
go build -o sub ./subscriber

./pub &

mkfifo pipe
./sub >pipe 2>&1 &

function check_output {
	received_msg='Received: &std_msgs_msg.String{Data:"gopher"}'
	while read -r line; do
		if [[ "$line" == *"$received_msg"* ]]; then
			exit 0
		fi
	done <pipe
	exit 1
}
export -f check_output

timeout 3 bash -c 'check_output'
`)
}
