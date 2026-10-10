/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"encoding/csv"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"unicode"
	"unicode/utf8"

	"github.com/kivilahtio/go-re/v0"
	"golang.org/x/exp/maps"
	"golang.org/x/exp/slices"
	"golang.org/x/tools/go/packages"
)

func goLicenseHeader(license string) string {
	if license == "" {
		return ""
	}
	return "/*\n" + license + "*/\n\n"
}

func ucFirst(s string) string {
	r, size := utf8.DecodeRuneInString(s)
	if r == utf8.RuneError {
		return s
	}
	return string(unicode.ToUpper(r)) + s[size:]
}

func snakeToCamel(in string) string {
	tmp := []rune(in)
	tmp[0] = unicode.ToUpper(tmp[0])
	for i := 0; i < len(tmp); i++ {
		if tmp[i] == '_' {
			tmp[i+1] = unicode.ToUpper(tmp[i+1])
			tmp = append(tmp[:i], tmp[i+1:]...)
			i--
		}
	}
	return string(tmp)
}

func camelToSnake(in string) string {
	tmp := []rune(in)
	sb := strings.Builder{}
	sb.Grow(len(tmp))

	ucSequenceLength := 0 //Special semantics for consecutive UC characters

	for i := 0; i < len(tmp); i++ {
		if unicode.IsUpper(tmp[i]) || (ucSequenceLength > 0 && unicode.IsNumber(tmp[i])) {
			ucSequenceLength++

			if i == 0 {
				sb.WriteRune(unicode.ToLower(tmp[i]))
			} else if ucSequenceLength == 1 {
				sb.WriteRune('_')
				sb.WriteRune(unicode.ToLower(tmp[i]))
			} else if i+1 >= len(tmp) {
				sb.WriteRune(unicode.ToLower(tmp[i]))
			} else if unicode.IsUpper(tmp[i+1]) || unicode.IsNumber(tmp[i+1]) {
				sb.WriteRune(unicode.ToLower(tmp[i]))
			} else {
				sb.WriteRune('_')
				sb.WriteRune(unicode.ToLower(tmp[i]))
			}
		} else {
			ucSequenceLength = 0
			sb.WriteRune(tmp[i])
		}
	}
	return sb.String()
}

func commentSerializer(lineComment string, preComments *strings.Builder) string {
	if preComments.Len() == 0 {
		return lineComment
	}
	defer preComments.Reset()
	if lineComment == "" {
		return preComments.String()
	}
	return lineComment + `. ` + preComments.String()
}

/*
The simple linux mkdir -p without all the Go-fuzz
*/
func mkdir_p(destFilePath string) (*os.File, error) {
	_, err := os.Stat(destFilePath)
	if errors.Is(err, os.ErrNotExist) {
		err = os.MkdirAll(filepath.Dir(destFilePath), os.ModePerm)
		if err != nil {
			return nil, err
		}
	}
	destFile, err := os.Create(destFilePath)
	if err != nil {
		return nil, err
	}
	return destFile, nil
}

func normalizeMsgDefaultArrayValue(defaultsField string) string {
	return re.Ss(defaultsField, `s!(?:^\[)|(?:\]$)!!gsm`) // So much fun with regexp love! Accurately trim leading and following [] without possible side-effects
}

func defaultValueSanitizer(ros2type string, defaultValue string) string {
	switch ros2type {
	// CSV parser removes the double quotes only, here we invoke the defaults parsing directly, and need to deal with double quotations manually
	case "string", "wstring", "U16String":
		if defaultValue != "" {
			re.S(&defaultValue, `s!(?:^")|(?:"$)!!gsm`)
		}
	}
	return defaultValueSanitizer_(ros2type, defaultValue)
}

func splitMsgDefaultArrayValues(ros2type string, defaultsField string) []string {
	defaultsField = normalizeMsgDefaultArrayValue(defaultsField)
	csv := csv.NewReader(strings.NewReader(defaultsField))
	csv.LazyQuotes = true
	csv.TrimLeadingSpace = true
	defaultValues, err := csv.Read()
	if err != nil && !errors.Is(err, io.EOF) {
		PrintErrf("%+v", err)
	}
	switch ros2type {
	// ROS2 string defaults CAN be quoted differently than how Golang MUST be quoted.
	case "string", "wstring", "U16String":
		for i := range defaultValues {
			defaultValues[i] = defaultValueSanitizer_(ros2type, defaultValues[i])
		}
	}

	return defaultValues
}

func defaultValueSanitizer_(ros2type, defaultValue string) string {
	if v, ok := canonicalPrimitiveValue(ros2type, strings.TrimSpace(defaultValue)); ok {
		return v
	}
	switch ros2type {
	// ROS2 string defaults CAN be quoted differently than how Golang MUST be quoted.
	case "string", "wstring", "U16String":
		if defaultValue != "" {
			re.S(&defaultValue, `s!(?:^')|(?:'$)!!gsm`)
			re.S(&defaultValue, `s!(?:\\)?"!\"!gsm`)
			re.S(&defaultValue, `s!(?:\\)?'!'!gsm`)
			re.S(&defaultValue, `s!(?:^)|(?:$)!"!gsm`)
		} else {
			defaultValue = "\"\""
		}
	}
	return defaultValue
}

func srvNameFromSrvMsgName(s string) string {
	re.S(&s, `s/_(Request|Response)$//`)
	return s
}

func actionNameFromActionMsgName(s string) string {
	re.S(&s, `s/_(Goal|Result|Feedback|SendGoal_Request|SendGoal_Response|GetResult_Request|GetResult_Response|FeedbackMessage)$//`)
	return s
}

func actionNameFromActionSrvName(s string) string {
	re.S(&s, `s/_(SendGoal|GetResult)$//`)
	return s
}

func cReturnCodeNameToGo(n string) string {
	re.S(&n, `s/^RCL_RET_//`)
	re.S(&n, `s/^RMW_RET_/RMW_/`)
	return snakeToCamel(strings.ToLower(n))
}

type stringSet map[string]struct{}

func (s stringSet) Add(strs ...string) {
	for _, str := range strs {
		s[str] = struct{}{}
	}
}

func (s stringSet) AddFrom(s2 stringSet) {
	for key := range s2 {
		s[key] = struct{}{}
	}
}

func (s stringSet) ToSlice() []string { return maps.Keys(s) }

func (s stringSet) ToSortedSlice() []string {
	vals := s.ToSlice()
	sort.Strings(vals)
	return vals
}

func actionHasSuffix(msg *ROS2Message, suffixes ...string) bool {
	if msg.Type == "action" {
		for _, suffix := range suffixes {
			if strings.HasSuffix(msg.Name, suffix) {
				return true
			}
		}
	}
	return false
}

func matchMsg(msg *ROS2Message, pkg, name string) bool {
	return msg.GoPackage() == pkg && msg.Name == name
}

func loadGoPkgDeps(pkgPaths ...string) (stringSet, error) {
	deps := stringSet{}
	if len(pkgPaths) > 0 {
		queries := slices.Clone(pkgPaths)
		for i := range queries {
			queries[i] = "pattern=" + queries[i]
		}
		pkgs, err := packages.Load(&packages.Config{
			Mode:  packages.NeedImports | packages.NeedDeps | packages.NeedName,
			Tests: true,
		}, queries...)
		if err != nil {
			return nil, err
		}
		packages.Visit(pkgs, func(pkg *packages.Package) bool {
			deps.Add(pkg.PkgPath)
			return true
		}, nil)
	}
	return deps, nil
}

func PrintErr(a ...any) { fmt.Fprint(os.Stderr, a...) }

func PrintErrln(a ...any) { fmt.Fprintln(os.Stderr, a...) }

func PrintErrf(format string, a ...any) { fmt.Fprintf(os.Stderr, format, a...) }

// canonicalPrimitiveValue returns value of a bool or integer type as a Go
// literal with the value rosidl gives it: rosidl reads booleans
// case-insensitively ("True", "1") and integers as Python's int() does
// (decimal first, so "010" is 10, then 0x/0o/0b prefixes). ok is false for
// other types and for values that do not parse, which are left as they are.
func canonicalPrimitiveValue(rosType, value string) (_ string, ok bool) {
	switch rosType {
	case "bool":
		switch strings.ToLower(value) {
		case "true", "1":
			return "true", true
		case "false", "0":
			return "false", true
		}
	case "byte", "char", "uint8", "uint16", "uint32", "uint64":
		u, err := strconv.ParseUint(value, 10, 64)
		if err != nil {
			u, err = strconv.ParseUint(value, 0, 64)
		}
		if err == nil {
			return strconv.FormatUint(u, 10), true
		}
	case "int8", "int16", "int32", "int64":
		i, err := strconv.ParseInt(value, 10, 64)
		if err != nil {
			i, err = strconv.ParseInt(value, 0, 64)
		}
		if err == nil {
			return strconv.FormatInt(i, 10), true
		}
	}
	return "", false
}
