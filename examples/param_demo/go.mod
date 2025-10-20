module rclgo_param_demo_pkg

go 1.22

require github.com/merlindrones/rclgo v0.0.0-20250701001034-bab9cbe02fd7

require (
	github.com/kr/pretty v0.3.1 // indirect
	gopkg.in/yaml.v3 v3.0.1 // indirect
)

// If you have a local checkout of rclgo you want to use instead of a tagged version:
replace github.com/merlindrones/rclgo => ../../
