package params

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/merlindrones/rclgo/pkg/rclgo"
)

// TestParameterSourcePriority verifies the parameter source priority order:
// CLI > Env Vars > YAML > Defaults
//
// This test sets a parameter from all four sources and verifies that the
// CLI override takes final precedence.
func TestParameterSourcePriority(t *testing.T) {
	// Set environment variable
	os.Setenv("RCL_PARAM_test_param", "200")
	defer os.Unsetenv("RCL_PARAM_test_param")

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("priority_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// STEP 1: Declare with default value (lowest priority)
	_, _ = mgr.Declare("test_param", Value{Kind: KindInt64, Int64: 100}, Descriptor{
		Description: "Test parameter for priority testing",
	})

	// Verify default value
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 100 {
		t.Errorf("After Declare: got %v, want 100 (default)", p.Value.Int64)
	}

	// STEP 2: Apply environment variables (overrides defaults)
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Verify env var overrode default
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 200 {
		t.Errorf("After ApplyEnvVars: got %v, want 200 (from env)", p.Value.Int64)
	}

	// STEP 3: Load from YAML (overrides env vars)
	tmpDir := t.TempDir()
	yamlPath := filepath.Join(tmpDir, "priority_test.yaml")
	yamlContent := `priority_test:
  ros__parameters:
    test_param: 300
`
	if err := os.WriteFile(yamlPath, []byte(yamlContent), 0644); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	if err := LoadYAML(mgr, "priority_test", yamlPath); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	// Verify YAML overrode env var
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 300 {
		t.Errorf("After LoadYAML: got %v, want 300 (from YAML)", p.Value.Int64)
	}

	// STEP 4: Apply CLI overrides (highest priority)
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "test_param:=400",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	if err := ApplyOverrides(mgr, "priority_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify CLI override took final precedence
	if p, ok := mgr.Get("test_param"); !ok || p.Value.Int64 != 400 {
		t.Errorf("After ApplyOverrides: got %v, want 400 (from CLI - highest priority)", p.Value.Int64)
	}
}

// TestParameterSourcePriority_PartialOverrides verifies that each source
// can override different parameters independently.
func TestParameterSourcePriority_PartialOverrides(t *testing.T) {
	// Set env vars for param2 and param3
	os.Setenv("RCL_PARAM_param2", "env_value2")
	os.Setenv("RCL_PARAM_param3", "env_value3")
	defer func() {
		os.Unsetenv("RCL_PARAM_param2")
		os.Unsetenv("RCL_PARAM_param3")
	}()

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("partial_priority_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// Declare 4 parameters with defaults
	_, _ = mgr.Declare("param1", Value{Kind: KindString, Str: "default1"}, Descriptor{})
	_, _ = mgr.Declare("param2", Value{Kind: KindString, Str: "default2"}, Descriptor{})
	_, _ = mgr.Declare("param3", Value{Kind: KindString, Str: "default3"}, Descriptor{})
	_, _ = mgr.Declare("param4", Value{Kind: KindString, Str: "default4"}, Descriptor{})

	// Apply env vars (overrides param2 and param3)
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// Create YAML that overrides param3 and param4
	tmpDir := t.TempDir()
	yamlPath := filepath.Join(tmpDir, "partial_test.yaml")
	yamlContent := `partial_priority_test:
  ros__parameters:
    param3: "yaml_value3"
    param4: "yaml_value4"
`
	if err := os.WriteFile(yamlPath, []byte(yamlContent), 0644); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	if err := LoadYAML(mgr, "partial_priority_test", yamlPath); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	// Apply CLI override for param4 only
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "param4:=cli_value4",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	if err := ApplyOverrides(mgr, "partial_priority_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify final values:
	// param1: default (no overrides)
	// param2: env_value2 (env var only)
	// param3: yaml_value3 (env var overridden by YAML)
	// param4: cli_value4 (env var → YAML → CLI)

	if p, ok := mgr.Get("param1"); !ok || p.Value.Str != "default1" {
		t.Errorf("param1: got %v, want 'default1' (default only)", p.Value.Str)
	}

	if p, ok := mgr.Get("param2"); !ok || p.Value.Str != "env_value2" {
		t.Errorf("param2: got %v, want 'env_value2' (env var)", p.Value.Str)
	}

	if p, ok := mgr.Get("param3"); !ok || p.Value.Str != "yaml_value3" {
		t.Errorf("param3: got %v, want 'yaml_value3' (YAML overrides env)", p.Value.Str)
	}

	if p, ok := mgr.Get("param4"); !ok || p.Value.Str != "cli_value4" {
		t.Errorf("param4: got %v, want 'cli_value4' (CLI overrides all)", p.Value.Str)
	}
}

// TestParameterSourcePriority_AllSources verifies the complete recommended
// usage pattern with all four sources.
func TestParameterSourcePriority_AllSources(t *testing.T) {
	// Set up environment
	os.Setenv("RCL_PARAM_fps", "60")
	os.Setenv("RCL_PARAM_resolution", "1080")
	defer func() {
		os.Unsetenv("RCL_PARAM_fps")
		os.Unsetenv("RCL_PARAM_resolution")
	}()

	// Init RCL context
	if err := rclgo.Init(nil); err != nil {
		t.Fatalf("rclgo.Init: %v", err)
	}
	defer rclgo.Uninit()

	// Create node and manager
	n, err := rclgo.NewNode("all_sources_test", "")
	if err != nil {
		t.Fatalf("NewNode: %v", err)
	}
	defer n.Close()

	mgr := &Manager{st: newStore(), n: n}

	// 1. Declare defaults
	_, _ = mgr.Declare("fps", Value{Kind: KindInt64, Int64: 15}, Descriptor{
		Description: "Camera frames per second",
	})
	_, _ = mgr.Declare("resolution", Value{Kind: KindInt64, Int64: 480}, Descriptor{
		Description: "Video resolution",
	})
	_, _ = mgr.Declare("quality", Value{Kind: KindString, Str: "medium"}, Descriptor{
		Description: "Video quality",
	})

	// 2. Apply environment variables
	if err := ApplyEnvVars(mgr); err != nil {
		t.Fatalf("ApplyEnvVars: %v", err)
	}

	// 3. Load YAML
	tmpDir := t.TempDir()
	yamlPath := filepath.Join(tmpDir, "all_sources.yaml")
	yamlContent := `all_sources_test:
  ros__parameters:
    resolution: 720
    quality: "high"
`
	if err := os.WriteFile(yamlPath, []byte(yamlContent), 0644); err != nil {
		t.Fatalf("WriteFile: %v", err)
	}

	if err := LoadYAML(mgr, "all_sources_test", yamlPath); err != nil {
		t.Fatalf("LoadYAML: %v", err)
	}

	// 4. Apply CLI overrides
	args, err := buildTestArgs([]string{
		"--ros-args",
		"-p", "fps:=120",
		"--",
	})
	if err != nil {
		t.Fatalf("buildTestArgs: %v", err)
	}

	if err := ApplyOverrides(mgr, "all_sources_test", args); err != nil {
		t.Fatalf("ApplyOverrides: %v", err)
	}

	// Verify final priority:
	// fps: 120 (CLI > Env[60] > Default[15])
	// resolution: 720 (YAML > Env[1080] > Default[480])
	// quality: "high" (YAML > Default["medium"])

	if p, ok := mgr.Get("fps"); !ok || p.Value.Int64 != 120 {
		t.Errorf("fps: got %v, want 120 (CLI highest priority)", p.Value.Int64)
	}

	if p, ok := mgr.Get("resolution"); !ok || p.Value.Int64 != 720 {
		t.Errorf("resolution: got %v, want 720 (YAML overrides env)", p.Value.Int64)
	}

	if p, ok := mgr.Get("quality"); !ok || p.Value.Str != "high" {
		t.Errorf("quality: got %v, want 'high' (YAML overrides default)", p.Value.Str)
	}
}
