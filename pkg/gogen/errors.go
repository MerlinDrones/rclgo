/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"os"
	"path/filepath"
	"strings"

	"github.com/kivilahtio/go-re/v0"
)

var errorTypesCFileMatchingRegexp string

func init() {
	prepareErrorTypesCFileMatchingRegexp()
}

/*
prepareErrorTypesCFileMatchingRegexp is a convenience function to be more easily able to define the C header files to look for error definitions without needing to fiddle with complex regexp
*/
func prepareErrorTypesCFileMatchingRegexp() {
	errorTypesCFileMatchingRegexp = "(" + strings.Join(cErrorTypeFiles, ")|(") + ")"
	re.R(&errorTypesCFileMatchingRegexp, `s!\!!\!!`)
	errorTypesCFileMatchingRegexp = "m!" + errorTypesCFileMatchingRegexp + "!"
}

func (g *Generator) GenerateROS2ErrorTypes() error {
	destFilePath := filepath.Join(g.config.DestPath, "pkg/rclgo/errortypes.gen.go")
	var errorTypes []*ROS2ErrorType
	parser := newErrorTypeParser()

	for _, includeLookupDir := range g.config.RootPaths {
		for tries := 0; tries < 10; tries++ {
			PrintErrf("Looking for rcl C include files to parse error definitions from '%s'\n", includeLookupDir)

			filepath.Walk(includeLookupDir, func(path string, info os.FileInfo, err error) error { //nolint:errcheck
				if err == nil && re.M(path, errorTypesCFileMatchingRegexp) {
					PrintErrf("Analyzing: %s\n", path)
					errorTypes, err = parser.parseFile(errorTypes, path)
					if err != nil {
						PrintErrf("Error converting ROS2 Errors from '%s' to '%s', error: %v\n", path, destFilePath, err)
					}
				}
				return nil
			})

			if len(errorTypes) == 0 {
				includeLookupDir = filepath.Join(includeLookupDir, "..")
				if includeLookupDir == "/" {
					break
				}
			} else {
				break
			}
		}
	}
	if len(errorTypes) == 0 {
		PrintErrf("Unable to find any rcl C error header files?\n")
		return nil
	}

	PrintErrf("Generating ROS2 Error definitions: %s\n", destFilePath)
	return g.generateGoFile(
		destFilePath,
		ros2ErrorCodes,
		templateData{
			"errorTypes":  errorTypes,
			"includes":    cErrorTypeFiles,
			"dedupFilter": parser.dedupFilter,
		},
	)
}

// errorTypeParser parses rcl and rmw error code definitions. Its state belongs
// to one generation run, so that generating twice in a process gives the same
// result.
type errorTypeParser struct {
	comments    strings.Builder   // Collect pre-field comments here to be included in the comments. Flushed on empty lines.
	dedupMap    map[string]string // Some RMW and RCL error codes overlap, so we need to deduplicate them from the dynamic type casting switch-case
	dedupFilter map[string]string // Entries ending up here actually filter template entries
}

func newErrorTypeParser() *errorTypeParser {
	return &errorTypeParser{
		dedupMap:    make(map[string]string, 1024),
		dedupFilter: make(map[string]string, 1024),
	}
}

func (p *errorTypeParser) parseFile(errorTypes []*ROS2ErrorType, path string) ([]*ROS2ErrorType, error) {
	content, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}

	for _, line := range strings.Split(string(content), "\n") {
		errType := p.parse(line)
		if errType != nil {
			errorTypes = append(errorTypes, errType)
		}
	}

	return errorTypes, nil
}

func (p *errorTypeParser) parse(row string) *ROS2ErrorType {
	if re.M(row, `m!
		^
		\#define\s+
		(?P<name>(?:RCL|RMW)_RET_\w+)\s+
		(?:(?P<int>\d+)|(?P<reference>\w+))\s*
		(?://\s*(?P<comment>.+))?
		\s*$
	!x`) {
		et := &ROS2ErrorType{
			Name:      re.R0.Z["name"],
			Rcl_ret_t: re.R0.Z["int"],
			Reference: re.R0.Z["reference"],
			Comment:   commentSerializer(re.R0.Z["comment"], &p.comments),
		}
		p.comments.Reset()
		p.updateDeduplicationMap(et.Rcl_ret_t, et.Name)
		return et

	} else if re.M(row, `m!/{2,}\s*(.+)$!`) { // this is a comment line
		if re.R0.S[1] != "" {
			p.comments.WriteString(re.R0.S[1])
		}

		return nil

	} else if row == "" { // do not process empty lines or comment lines
		p.comments.Reset()

		return nil

	}
	return nil
}

func (p *errorTypeParser) updateDeduplicationMap(rcl_ret_t string, name string) {
	_, ok := p.dedupMap[rcl_ret_t]
	if ok {
		p.dedupFilter[name] = rcl_ret_t // If the rcl_ret_t was taken, deduplicate
	} else {
		p.dedupMap[rcl_ret_t] = name // On first match, simply register that the map key is taken
	}
}
