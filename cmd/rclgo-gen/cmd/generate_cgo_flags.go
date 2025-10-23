/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package cmd

import (
	"fmt"
	"os"

	"github.com/merlindrones/rclgo/pkg/gogen"
	"github.com/spf13/cobra"
)

func validateGenerateCgoFlagsArgs(cmd *cobra.Command, _ []string) error {
	rootPaths := getRootPaths(cmd)
	if len(rootPaths) == 0 {
		if os.Getenv("AMENT_PREFIX_PATH") == "" {
			return fmt.Errorf("You haven't sourced your ROS2 environment! Cannot autodetect --root-path. Source your ROS2 or pass --root-path")
		}
		return fmt.Errorf("root-path is required")
	}

	distro := os.Getenv("ROS_DISTRO")
	if getBool(cmd, "ignore-ros-distro-mismatch") {
		if distro != correctDistro {
			gogen.PrintErrf("NOTE: Environment variable ROS_DISTRO is set to %q, generating files for %q\n", distro, correctDistro)
		}
	} else if distro != correctDistro {
		return fmt.Errorf("ROS_DISTRO should be set to %q", correctDistro)
	}

	return nil
}

var generateCgoFlagsCmd = &cobra.Command{
	Use:   "generate-cgo-flags",
	Short: "Generate cgo-flags.env file for ROS2 workspace development",
	Long: `Generate a cgo-flags.env file containing CGO_CFLAGS and CGO_LDFLAGS
for building Go programs that use ROS 2.

This command is useful for ROS 2 workspace development where you need to
include both the base ROS 2 installation and workspace overlay paths.

Example usage:

  # Generate cgo-flags.env for a workspace with custom packages
  rclgo-gen generate-cgo-flags \
    --root-path /opt/ros/humble \
    --root-path $ROS_WS/install \
    --output $ROS_WS/cgo-flags.env \
    --include-package px4_msgs \
    --include-package swarmos_msgs

  # Generate to stdout
  rclgo-gen generate-cgo-flags \
    --root-path /opt/ros/humble \
    --root-path $ROS_WS/install \
    --output - \
    --include-package px4_msgs

The --include-package flags are optional and only used to ensure those
packages are scanned for C dependencies. The generated cgo-flags.env will
include all standard ROS 2 packages plus any custom packages found.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		// Get output path from --output flag instead of --cgo-flags-path
		outputPath := getString(cmd, "output")

		config, err := getGogenConfig(cmd)
		if err != nil {
			return err
		}

		// Override CGOFlagsPath with the output flag value
		config.CGOFlagsPath = outputPath

		gen := gogen.New(config)

		// Scan for packages to populate cImportsByPkgAndType
		gen.FindPackagesForCgoFlags()

		if err := gen.GenerateCGOFlags(); err != nil {
			return fmt.Errorf("failed to generate CGO flags: %w", err)
		}
		return nil
	},
	Args: validateGenerateCgoFlagsArgs,
}

func init() {
	rootCmd.AddCommand(generateCgoFlagsCmd)

	// Only configure flags relevant to CGO generation
	generateCgoFlagsCmd.PersistentFlags().StringArrayP("root-path", "r", []string{os.Getenv("AMENT_PREFIX_PATH")}, "Root lookup path for ROS2 packages. If ROS2 environment is sourced, is autodetected.")
	generateCgoFlagsCmd.PersistentFlags().StringP("output", "o", "cgo-flags.env", `Path to output file. Use "-" for stdout.`)
	generateCgoFlagsCmd.PersistentFlags().StringArray("include-package", nil, "Include packages matching a regex for dependency scanning. Can be passed multiple times.")
	generateCgoFlagsCmd.PersistentFlags().Bool("ignore-ros-distro-mismatch", false, "If true, ignores possible mismatches in sourced and supported ROS distro")

	// Override the cgo-flags-path to use --output flag
	generateCgoFlagsCmd.PersistentFlags().String("cgo-flags-path", "", "")
	generateCgoFlagsCmd.PersistentFlags().MarkHidden("cgo-flags-path")

	bindPFlags(generateCgoFlagsCmd)
}
