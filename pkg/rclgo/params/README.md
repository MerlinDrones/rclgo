# rclgo Parameters Package

Comprehensive ROS 2 parameter support for rclgo with full rclcpp/rclpy parity.

## Features

- ✅ Declare/undeclare parameters with constraints
- ✅ Get/set parameters with validation
- ✅ Parameter events publishing (`/parameter_events`)
- ✅ ROS 2 CLI compatibility (`ros2 param get/set/list/describe`)
- ✅ YAML configuration file loading
- ✅ CLI parameter overrides (`--ros-args -p param:=value`)
- ✅ Environment variable overrides (`RCL_PARAM_name=value`)
- ✅ OnSet callbacks for parameter changes
- ✅ Type constraints (min/max, allowed values)
- ✅ Read-only parameters
- ✅ `/use_sim_time` integration

## Parameter Priority Order

Parameters can be set from multiple sources. The priority order (highest to lowest) is:

1. **CLI overrides** (`--ros-args -p param:=value`) - Highest priority
2. **Environment variables** (`RCL_PARAM_name=value`)
3. **YAML files** (`--params-file config.yaml`)
4. **Declared defaults** (in code) - Lowest priority

## Quick Start

### Basic Usage

```go
package main

import (
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

func main() {
	// Parse command-line arguments
	rclArgs, _, _ := rclgo.ParseArgs(os.Args[1:])

	// Initialize RCL
	rclgo.Init(rclArgs)
	defer rclgo.Uninit()

	// Create node
	node, _ := rclgo.NewNode("my_node", "")
	defer node.Close()

	// Create parameter manager
	mgr, _ := params.NewManager(node)

	// Declare parameters with defaults
	mgr.Declare("camera.fps", params.Value{Kind: params.KindInt64, Int64: 30},
		params.Descriptor{
			Description: "Camera frames per second",
			MinInt: intPtr(1),
			MaxInt: intPtr(120),
		})

	// Apply parameter sources in priority order
	params.ApplyEnvVars(mgr)  // Environment variables
	params.LoadYAML(mgr, "my_node", "/path/to/config.yaml")  // YAML
	params.ApplyOverrides(mgr, node.Name(), rclArgs)  // CLI (highest priority)

	// Get parameter value
	if p, ok := mgr.Get("camera.fps"); ok {
		fps := p.Value.Int64
		// Use fps value...
	}
}

func intPtr(i int64) *int64 { return &i }
```

## Parameter Sources

### 1. Declare Defaults (Code)

```go
// Scalar types
mgr.Declare("enable_feature", params.Value{Kind: params.KindBool, Bool: false},
	params.Descriptor{Description: "Enable experimental feature"})

mgr.Declare("max_speed", params.Value{Kind: params.KindDouble, Double: 1.0},
	params.Descriptor{
		Description: "Maximum speed (m/s)",
		MinDouble: floatPtr(0.0),
		MaxDouble: floatPtr(5.0),
	})

// Array types
mgr.Declare("waypoints", params.Value{Kind: params.KindDoubleArray, Doubles: []float64{0, 0, 0}},
	params.Descriptor{Description: "Navigation waypoints"})
```

### 2. Environment Variables

Set parameters via environment variables using the `RCL_PARAM_` prefix:

```bash
# Scalar values
export RCL_PARAM_camera.fps=60
export RCL_PARAM_debug_mode=true
export RCL_PARAM_robot_name="explorer"

# Array values
export RCL_PARAM_thresholds="[0.1,0.5,0.9]"
export RCL_PARAM_tags="[front,rear,left]"

# Run node (env vars auto-applied)
./my_node
```

In code:

```go
// Apply environment variables
if err := params.ApplyEnvVars(mgr); err != nil {
	log.Printf("Failed to apply env vars: %v", err)
}
```

**Type Inference:**
- If parameter already declared: uses existing type
- If not declared: infers from string value
  - `"true"`/`"false"` → bool
  - Numeric with decimal → double
  - Numeric without decimal → int64
  - Array syntax `[a,b,c]` → array type
  - Otherwise → string

### 3. YAML Configuration Files

Create a YAML file `config.yaml`:

```yaml
my_node:
  ros__parameters:
    camera.fps: 30
    camera.resolution: 1920
    camera.exposure: 0.015
    debug_mode: false
    tags: ["camera1", "front", "rgb"]
```

Load in code:

```go
if err := params.LoadYAML(mgr, "my_node", "/path/to/config.yaml"); err != nil {
	log.Printf("Failed to load YAML: %v", err)
}
```

Or pass via command line:

```bash
./my_node --ros-args --params-file /path/to/config.yaml --
```

### 4. CLI Overrides (Highest Priority)

Override any parameter from the command line:

```bash
# Single parameter
./my_node --ros-args -p camera.fps:=60 --

# Multiple parameters
./my_node --ros-args \
  -p camera.fps:=60 \
  -p debug_mode:=true \
  -p robot_name:=explorer \
  --

# Array parameters
./my_node --ros-args -p thresholds:="[0.1,0.5,0.9]" --
```

In code:

```go
rclArgs, _, _ := rclgo.ParseArgs(os.Args[1:])
rclgo.Init(rclArgs)

// ... create node and manager ...

if err := params.ApplyOverrides(mgr, node.Name(), rclArgs); err != nil {
	log.Printf("Failed to apply CLI overrides: %v", err)
}
```

## Complete Example with All Sources

```go
package main

import (
	"log"
	"os"

	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/params"
)

func main() {
	// 1. Parse CLI args
	rclArgs, restArgs, _ := rclgo.ParseArgs(os.Args[1:])

	// 2. Initialize
	rclgo.Init(rclArgs)
	defer rclgo.Uninit()

	node, _ := rclgo.NewNode("camera_node", "")
	defer node.Close()

	mgr, _ := params.NewManager(node)

	// 3. Declare defaults (lowest priority)
	mgr.Declare("fps", params.Value{Kind: params.KindInt64, Int64: 15},
		params.Descriptor{
			Description: "Camera FPS",
			MinInt: intPtr(1),
			MaxInt: intPtr(120),
		})

	mgr.Declare("quality", params.Value{Kind: params.KindString, Str: "medium"},
		params.Descriptor{
			Description: "Video quality",
			AllowedStrings: []string{"low", "medium", "high"},
		})

	// 4. Apply environment variables
	//    e.g., RCL_PARAM_fps=30
	if err := params.ApplyEnvVars(mgr); err != nil {
		log.Printf("Warning: %v", err)
	}

	// 5. Load YAML config
	//    e.g., --params-file config.yaml
	if yamlPath := parseParamsFile(restArgs); yamlPath != "" {
		if err := params.LoadYAML(mgr, node.Name(), yamlPath); err != nil {
			log.Printf("Warning: %v", err)
		}
	}

	// 6. Apply CLI overrides (highest priority)
	//    e.g., -p fps:=60
	if err := params.ApplyOverrides(mgr, node.Name(), rclArgs); err != nil {
		log.Printf("Warning: %v", err)
	}

	// 7. Use parameters
	fps, _ := mgr.Get("fps")
	quality, _ := mgr.Get("quality")

	log.Printf("Running with fps=%d, quality=%s", fps.Value.Int64, quality.Value.Str)

	// Your application logic...
}

func parseParamsFile(args []string) string {
	for i, arg := range args {
		if arg == "--params-file" && i+1 < len(args) {
			return args[i+1]
		}
	}
	return ""
}

func intPtr(i int64) *int64 { return &i }
```

## Usage Examples

### Running with Different Parameter Sources

```bash
# Default values only
./my_node

# With environment variables
RCL_PARAM_fps=30 RCL_PARAM_debug=true ./my_node

# With YAML file
./my_node --params-file config.yaml

# With CLI overrides
./my_node --ros-args -p fps:=60 -p quality:=high --

# Combining all sources (priority: CLI > YAML > Env > Defaults)
RCL_PARAM_fps=30 ./my_node --params-file config.yaml --ros-args -p fps:=120 --
# Result: fps=120 (CLI wins)
```

### ROS 2 CLI Interaction

```bash
# List parameters
ros2 param list

# Get parameter value
ros2 param get /camera_node fps

# Set parameter value
ros2 param set /camera_node fps 60

# Describe parameter (shows constraints)
ros2 param describe /camera_node fps

# Dump all parameters to YAML
ros2 param dump /camera_node > params.yaml
```

## Parameter Constraints

### Numeric Ranges

```go
mgr.Declare("temperature", params.Value{Kind: params.KindDouble, Double: 20.0},
	params.Descriptor{
		Description: "Operating temperature (°C)",
		MinDouble: floatPtr(-10.0),
		MaxDouble: floatPtr(50.0),
	})
```

### Allowed String Values

```go
mgr.Declare("mode", params.Value{Kind: params.KindString, Str: "auto"},
	params.Descriptor{
		Description: "Operating mode",
		AllowedStrings: []string{"auto", "manual", "remote"},
	})
```

### Read-Only Parameters

```go
mgr.Declare("version", params.Value{Kind: params.KindString, Str: "1.0.0"},
	params.Descriptor{
		Description: "Software version",
		ReadOnly: true,  // Cannot be changed after declaration
	})
```

## OnSet Callbacks

React to parameter changes:

```go
mgr.OnSet(func(params []params.Parameter) params.SetResult {
	for _, p := range params {
		if p.Name == "fps" && p.Value.Int64 > 100 {
			log.Printf("Warning: High FPS setting: %d", p.Value.Int64)
		}
	}
	// Accept all changes
	return params.SetResult{Successful: true}
})
```

Reject invalid changes:

```go
mgr.OnSet(func(params []params.Parameter) params.SetResult {
	for _, p := range params {
		if p.Name == "critical_param" && !validateCritical(p.Value) {
			return params.SetResult{
				Successful: false,
				Reason: "Invalid critical parameter value",
			}
		}
	}
	return params.SetResult{Successful: true}
})
```

## Parameter Types

### Supported Types

- `KindBool` - Boolean
- `KindInt64` - 64-bit integer
- `KindDouble` - Double-precision float
- `KindString` - String
- `KindBytes` - Byte array
- `KindBoolArray` - Boolean array
- `KindInt64Array` - Integer array
- `KindDoubleArray` - Double array
- `KindStringArray` - String array

### Creating Values

```go
// Scalars
boolVal := params.Value{Kind: params.KindBool, Bool: true}
intVal := params.Value{Kind: params.KindInt64, Int64: 42}
doubleVal := params.Value{Kind: params.KindDouble, Double: 3.14}
strVal := params.Value{Kind: params.KindString, Str: "hello"}

// Arrays
boolArrVal := params.Value{Kind: params.KindBoolArray, Bools: []bool{true, false}}
intArrVal := params.Value{Kind: params.KindInt64Array, Int64s: []int64{1, 2, 3}}
doubleArrVal := params.Value{Kind: params.KindDoubleArray, Doubles: []float64{1.1, 2.2}}
strArrVal := params.Value{Kind: params.KindStringArray, Strs: []string{"a", "b"}}
```

## Best Practices

1. **Always declare defaults in code** - Ensures parameters exist even without external config
2. **Use descriptive parameter names** - Prefer `camera.fps` over `fps` for clarity
3. **Add descriptions to all parameters** - Helps with `ros2 param describe`
4. **Set appropriate constraints** - Use min/max ranges and allowed values
5. **Apply sources in correct order**:
   ```go
   // Recommended order
   params.ApplyEnvVars(mgr)          // 1. Env vars
   params.LoadYAML(mgr, ...)         // 2. YAML
   params.ApplyOverrides(mgr, ...)   // 3. CLI (highest priority)
   ```
6. **Use OnSet callbacks for validation** - Enforce complex constraints
7. **Handle errors gracefully** - Parameter loading should not crash your node

## Testing

The params package includes comprehensive tests:

```bash
# Run all params tests
source /opt/ros/humble/setup.bash
go test ./pkg/rclgo/params -v

# Test specific functionality
go test ./pkg/rclgo/params -v -run TestApplyEnvVars
go test ./pkg/rclgo/params -v -run TestApplyOverrides
go test ./pkg/rclgo/params -v -run TestParameterSourcePriority
```

## See Also

- [param_demo example](../../../examples/param_demo/) - Complete working example
- [ROS 2 Parameters Design](http://design.ros2.org/articles/ros_parameters.html)
- [ROS 2 Command Line Args](http://design.ros2.org/articles/ros_command_line_arguments.html)
