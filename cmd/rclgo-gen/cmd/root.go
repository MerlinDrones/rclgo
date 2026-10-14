/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package cmd

import (
	"errors"
	"fmt"
	"os"

	homedir "github.com/mitchellh/go-homedir"
	"github.com/spf13/cobra"
	"github.com/spf13/viper"
)

// rootCmd represents the base command when called without any subcommands
var rootCmd = &cobra.Command{
	Use:   "rclgo-gen",
	Short: "ROS2 client library in Golang - ROS2 Message generator",
	Long:  `Call this program to generate Go types for the ROS2 messages found in your ROS2 environment.`,
	// Execute prints the error once, and exits with status 1.
	SilenceErrors: true,
	// Usage is shown for bad flags and arguments, not for failures while
	// generating.
	PersistentPreRun: func(cmd *cobra.Command, _ []string) { cmd.SilenceUsage = true },
}

// Execute adds all child commands to the root command and sets flags appropriately.
// This is called by main.main(). It only needs to happen once to the rootCmd.
func Execute() {
	cobra.CheckErr(rootCmd.Execute())
}

func init() {
	cobra.OnInitialize(initConfig)

	rootCmd.PersistentFlags().StringP("config-file", "c", "", "config file (default is $HOME/.rclgo.yaml)")
	viper.BindPFlags(rootCmd.PersistentFlags())
	viper.BindPFlags(rootCmd.LocalFlags())
}

// initConfig reads in config file and ENV variables if set.
func initConfig() {
	if viper.GetString("config-file") != "" {
		// Use config file from the flag.
		viper.SetConfigFile(viper.GetString("config-file"))
	} else {
		// Find home directory.
		home, err := homedir.Dir()
		cobra.CheckErr(err)

		// Search config in home directory with name ".rclgo" (without extension).
		viper.AddConfigPath(home)
		viper.SetConfigName(".rclgo")
	}

	viper.AutomaticEnv() // read in environment variables that match

	// Read the config file. A missing default ~/.rclgo is fine; a config file
	// given with --config-file that can't be read, or any malformed one, is
	// an error rather than silently ignored.
	err := viper.ReadInConfig()
	var notFound viper.ConfigFileNotFoundError
	switch {
	case err == nil:
		fmt.Fprintln(os.Stderr, "Using config file:", viper.ConfigFileUsed())
	case viper.GetString("config-file") == "" && errors.As(err, &notFound):
	default:
		cobra.CheckErr(fmt.Errorf("config file: %w", err))
	}
}
