/*
This file is part of rclgo

Copyright © 2021 Technology Innovation Institute, United Arab Emirates

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package gogen

import (
	"errors"
	"fmt"
	"math"
	"regexp"
	"strconv"
	"strings"
	"unicode/utf8"

	"github.com/kivilahtio/go-re/v0"
)

var goKeywords = map[string]struct{}{
	"break": {}, "default": {}, "func": {}, "interface": {}, "select": {},
	"case": {}, "defer": {}, "go": {}, "map": {}, "struct": {},
	"chan": {}, "else": {}, "goto": {}, "package": {}, "switch": {},
	"const": {}, "fallthrough": {}, "if": {}, "range": {}, "type": {},
	"continue": {}, "for": {}, "import": {}, "return": {}, "var": {},
}

func cName(rosName string) string {
	if _, ok := goKeywords[rosName]; ok {
		return "_" + rosName
	}
	return rosName
}

type parser struct {
	config *Config
	// Collect pre-field comments here to be included in the comments. Flushed
	// on empty lines.
	ros2messagesCommentsBuffer strings.Builder
}

func ParseMessage(config *Config, content string) (*ROS2Message, error) {
	p := &parser{config: config}
	msg := ROS2MessageNew("", "")
	if err := p.ParseROS2Message(msg, content); err != nil {
		return nil, err
	}
	return msg, nil
}

// ParseROS2Message parses a message definition.
func (p *parser) ParseROS2Message(res *ROS2Message, content string) error {
	return p.parseSections(content, res)
}

func (p *parser) ParseService(service *ROS2Service, source string) error {
	return p.parseSections(source, service.Request, service.Response)
}

func (p *parser) ParseAction(action *ROS2Action, source string) error {
	err := p.parseSections(source, action.Goal, action.Result, action.Feedback)
	if err != nil {
		return err
	}
	p.addImport(action.SendGoal.Request, "unique_identifier_msgs")
	action.SendGoal.Request.Fields = []*ROS2Field{
		p.goalIDField(),
		p.actionLocalField("goal", "Goal", action, action.Goal),
	}
	p.addImport(action.SendGoal.Response, "builtin_interfaces")
	action.SendGoal.Response.Fields = []*ROS2Field{
		p.primitiveField("accepted", "Accepted", "bool", "bool"),
		{
			RosName: "stamp",
			CName:   "stamp",
			GoName:  "Stamp",

			PkgName:   "builtin_interfaces",
			GoPkgName: "builtin_interfaces_msg",

			RosType: "Time",
			CType:   "Time",
			GoType:  "Time",
		},
	}
	p.addImport(action.GetResult.Request, "unique_identifier_msgs")
	action.GetResult.Request.Fields = []*ROS2Field{p.goalIDField()}
	action.GetResult.Response.Fields = []*ROS2Field{
		p.primitiveField("status", "Status", "int8_t", "int8"),
		p.actionLocalField("result", "Result", action, action.Result),
	}
	p.addImport(action.FeedbackMessage, "unique_identifier_msgs")
	action.FeedbackMessage.Fields = []*ROS2Field{
		p.goalIDField(),
		p.actionLocalField("feedback", "Feedback", action, action.Feedback),
	}
	return nil
}

func (p *parser) goalIDField() *ROS2Field {
	return &ROS2Field{
		RosName: "goal_id",
		CName:   "goal_id",
		GoName:  "GoalID",

		PkgName:   "unique_identifier_msgs",
		GoPkgName: "unique_identifier_msgs_msg",

		RosType: "UUID",
		CType:   "UUID",
		GoType:  "UUID",
	}
}

func (p *parser) primitiveField(cname, goname, ctype, gotype string) *ROS2Field {
	return &ROS2Field{
		RosName: cname,
		CName:   cname,
		GoName:  goname,

		RosType: gotype,
		CType:   ctype,
		GoType:  gotype,
	}
}

func (p *parser) actionLocalField(cname, goname string, action *ROS2Action, msg *ROS2Message) *ROS2Field {
	return &ROS2Field{
		RosName: cname,
		CName:   cname,
		GoName:  goname,

		PkgName:    action.Package,
		GoPkgName:  action.GoPackage(),
		PkgIsLocal: true,

		RosType: msg.Name,
		CType:   msg.Name,
		GoType:  msg.Name,
	}
}

func (p *parser) addImportSpecial(msg *ROS2Message, cPkg, goPkg, goImport string) {
	if goImport == "" {
		goImport = goPkg
	}
	if msg.GoImports[goImport] == "" {
		msg.GoImports[goImport] = goPkg
		msg.CImports.Add(cPkg)
	}
}

func (p *parser) addImport(msg *ROS2Message, pkg string) {
	goImport := p.config.MessageModulePrefix + "/" + pkg + "/msg"
	p.addImportSpecial(msg, pkg, pkg+"_msg", goImport)
}

func (p *parser) parseSections(source string, sections ...*ROS2Message) error {
	current := 0
	var errs []error
	for i, line := range strings.Split(source, "\n") {
		line = strings.TrimSpace(line)
		if line == "---" {
			current++
			if current >= len(sections) {
				return errors.Join(append(errs, fmt.Errorf("line %d: too many sections", i+1))...)
			}
		} else if err := p.parseLine(sections[current], line); err != nil {
			errs = append(errs, fmt.Errorf("error on line %d: %w", i+1, err))
		}
	}
	return errors.Join(errs...)
}

func (p *parser) parseLine(msg *ROS2Message, line string) error {
	obj, err := p.parseMessageLine(line, msg)
	if err != nil {
		return err
	}

	switch obj := obj.(type) {
	case *ROS2Constant:
		msg.Constants = append(msg.Constants, obj)
	case *ROS2Field:
		msg.Fields = append(msg.Fields, obj)
		switch obj.PkgName {
		case "":
		case ".":
		case "time":
			msg.GoImports["time"] = ""
		case "primitives":
			msg.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/"+obj.PkgName] = obj.GoPkgName
		default:
			msg.GoImports[p.config.MessageModulePrefix+"/"+obj.PkgName+"/msg"] = obj.GoPkgName
			msg.CImports.Add(obj.PkgName)
		}
	case nil:
	default:
		return fmt.Errorf("couldn't parse the input row '%s'", line)
	}
	return nil
}

func (p *parser) parseMessageLine(testRow string, ros2msg *ROS2Message) (interface{}, error) {
	re.R(&testRow, `m!^#\s*(.*)$!`) // Extract comments from comment-only lines to be included in the pre-field comments
	if re.R0.Matches > 0 {
		if re.R0.S[1] != "" {
			p.ros2messagesCommentsBuffer.WriteString(re.R0.S[1])
		}
		return nil, nil
	}
	if testRow == "" { // do not process empty lines or comment lines
		p.ros2messagesCommentsBuffer.Reset()
		return nil, nil
	}
	// As rosidl_adapter does, a comment starts at the first '#' of the line,
	// even right after a value ("int32 x 5#c") or inside quotes.
	comment, hasComment := "", false
	if i := strings.IndexByte(testRow, '#'); i >= 0 {
		comment, hasComment = strings.TrimSpace(strings.TrimLeft(testRow[i:], "#")), true
		testRow = strings.TrimSpace(testRow[:i])
	}

	typeChar, capture := isRowConstantOrField(testRow)
	if hasComment && capture != nil {
		capture["comment"] = comment
	}
	switch typeChar {
	case 'c':
		con, err := p.ParseROS2MessageConstant(capture)
		if err != nil {
			return nil, fmt.Errorf("invalid constant '%s': %w", testRow, err)
		}
		return con, nil
	case 'f':
		f, err := p.ParseROS2MessageField(capture, ros2msg)
		if err != nil {
			return nil, fmt.Errorf("invalid field '%s': %w", testRow, err)
		}
		f.GoName = uniqueFieldGoName(f.GoName, ros2msg)
		return f, nil
	}
	return nil, fmt.Errorf("couldn't parse the input row as either ROS2 Field or Constant? input '%s'", testRow)
}

func isRowConstantOrField(textRow string) (byte, map[string]string) {
	re.R(&textRow, `m!
	# This regex might be overly complex for constant parsing but it works. It was originally a copy-paste of the ROS2 field-parser
	^
	(?:(?P<package>\w+)/)?
	(?P<type>\w+)
	(?P<array>\[(?P<bounded><=)?(?P<size>\d*)\])?
	\s+
	(?P<field>\w+)
	\s*=\s*
	(?P<default>[^#]+)?
	(?:\s*\#\s*(?P<comment>.*))?
	$
	!x`)
	if re.R0.Matches > 0 {
		return 'c', re.R0.Z
	}

	re.R(&textRow, `m!
	^(?:(?P<package>\w+)/)?
	(?P<type>\w+)
	(?P<boundedString><=\d*)?    # Special case for bounded strings
	(?P<array>\[(?P<bounded><=)?(?P<size>\d*)\])?
	\s+
	(?P<field>\w+)
	(?:\s+(?P<default>[^#]+))?
	(?:\s+\#\s*(?P<comment>.*))?
	$
	!x`)
	if re.R0.Matches > 0 {
		return 'f', re.R0.Z
	}

	return 'e', nil
}

func (p *parser) ParseROS2MessageConstant(capture map[string]string) (*ROS2Constant, error) {
	if capture["package"] != "" || capture["array"] != "" {
		return nil, errors.New("constants must have a primitive, non-array type")
	}
	if !validConstantName.MatchString(capture["field"]) {
		return nil, fmt.Errorf("invalid constant name '%s': must match %s", capture["field"], validConstantName)
	}
	d := &ROS2Constant{
		RosType: capture["type"],
		RosName: capture["field"],
		Value:   strings.TrimSpace(capture["default"]),
		Comment: commentSerializer(capture["comment"], &p.ros2messagesCommentsBuffer),
	}

	t, ok := primitiveTypeMappings[d.RosType]
	if !ok {
		d.GoType = fmt.Sprintf("<MISSING translation from ROS2 Constant type '%s'>", d.RosType)
		return d, fmt.Errorf("Unknown ROS2 Constant type '%s'\n", d.RosType)
	}
	if err := checkPrimitiveValue(d.RosType, d.Value, 0); err != nil {
		return nil, err
	}
	d.GoType = t.GoType
	d.PkgName = t.PackageName
	return d, nil
}

func (p *parser) ParseROS2MessageField(capture map[string]string, ros2msg *ROS2Message) (*ROS2Field, error) {
	capture["default"] = strings.TrimSpace(capture["default"])
	if err := checkField(capture); err != nil {
		return nil, err
	}
	size, err := strconv.ParseInt(capture["size"], 10, 32)
	if err != nil && capture["size"] != "" {
		return nil, err
	}
	if capture["boundedString"] != "" &&
		!(capture["package"] == "" && capture["type"] == "string") {
		return nil, errors.New("the only base type that supports an upper boundary is string")
	}
	if capture["bounded"] != "" {
		capture["array"] = strings.Replace(capture["array"], capture["bounded"]+capture["size"], "", 1)
		capture["bounded"] += capture["size"]
		size = 0
	}
	f := &ROS2Field{
		Comment:      commentSerializer(capture["comment"], &p.ros2messagesCommentsBuffer),
		GoName:       snakeToCamel(capture["field"]),
		RosName:      capture["field"],
		CName:        cName(capture["field"]),
		RosType:      capture["type"],
		TypeArray:    capture["array"],
		ArrayBounded: capture["bounded"],
		ArraySize:    int(size),
		DefaultValue: capture["default"],
		PkgName:      capture["package"],
	}

	f.PkgName, f.CType, f.GoType = translateROS2Type(f, ros2msg)
	f.GoPkgName = f.PkgName
	switch f.PkgName {
	case "", "time", "primitives":
	case ".":
		if ros2msg.Type == "msg" {
			f.PkgIsLocal = true
		} else {
			f.PkgName = ros2msg.Package
			f.GoPkgName = ros2msg.Package + "_msg"
		}
	default:
		f.GoPkgName = f.PkgName + "_msg"
	}
	// Prepopulate extra Go imports
	p.cSerializationCode(f, ros2msg)
	p.goSerializationCode(f, ros2msg)

	return f, nil
}

func translateROS2Type(f *ROS2Field, m *ROS2Message) (pkgName string, cType string, goType string) {
	t, ok := primitiveTypeMappings[f.RosType]
	if ok {
		f.RosType = t.RosType
		return t.PackageName, t.CType, t.GoType
	}

	if f.PkgName == "" && m.Type != "msg" {
		return m.Package, f.RosType, f.RosType
	}

	// explicit package
	if f.PkgName != "" {
		// type of same package
		if f.PkgName == m.Package {
			return ".", f.RosType, f.RosType
		}

		// type of other package
		return f.PkgName, f.RosType, f.RosType
	}

	// implicit package, type of std_msgs
	if m.Package != "std_msgs" {
		switch f.RosType {
		case "Bool", "ColorRGBA",
			"Duration", "Empty", "Float32MultiArray", "Float32",
			"Float64MultiArray", "Float64", "Header", "Int8MultiArray",
			"Int8", "Int16MultiArray", "Int16", "Int32MultiArray", "Int32",
			"Int64MultiArray", "Int64", "MultiArrayDimension", "MultiarrayLayout",
			"String", "Time", "UInt8MultiArray", "UInt8", "UInt16MultiArray", "UInt16",
			"UInt32MultiArray", "UInt32", "UInt64MultiArray", "UInt64":
			return "std_msgs", f.RosType, f.RosType
		}
	}

	// These are not actually primitive types, but same-package complex types.
	return ".", f.RosType, f.RosType
}

func (p *parser) cSerializationCode(f *ROS2Field, m *ROS2Message) string {
	if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName != "" && f.PkgIsLocal {
		// Complex value Array local package reference
		return ucFirst(f.RosType) + `__Array_into_C(mem.` + f.CName + `[:], m.` + f.GoName + `[:])`

	} else if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName != "" && !f.PkgIsLocal {
		// Complex value Array remote package reference
		return `cSlice_` + f.RosName + ` := mem.` + f.CName + `[:]
	` + f.GoPkgReference() + ucFirst(f.RosType) + `__Array_into_C(*(*[]` + f.GoPkgReference() + `C` + ucFirst(f.RosType) + `)(unsafe.Pointer(&cSlice_` + f.RosName + `)), m.` + f.GoName + `[:])`
	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName != "" && f.PkgIsLocal {
		// Complex value Slice local package reference
		return ucFirst(f.RosType) + `__Sequence_into_C(&mem.` + f.CName + `, m.` + f.GoName + `)`

	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName != "" && !f.PkgIsLocal {
		// Complex value Slice remote package reference
		return f.GoPkgReference() + ucFirst(f.RosType) + `__Sequence_into_C((*` + f.GoPkgReference() + `C` + ucFirst(f.RosType) + `__Sequence)(unsafe.Pointer(&mem.` + f.CName + `)), m.` + f.GoName + `)`

	} else if f.TypeArray == "" && f.PkgName != "" {
		// Complex value single
		return f.GoPkgReference() + f.GoType + "TypeSupport.AsCStruct(unsafe.Pointer(&mem." + f.CName + "), &m." + f.GoName + ")"

	} else if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName == "" {
		// Primitive value Array
		m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
		return `cSlice_` + f.RosName + ` := mem.` + f.CName + `[:]
	` + `primitives.` + ucFirst(f.RosType) + `__Array_into_C(*(*[]primitives.C` + ucFirst(f.RosType) + `)(unsafe.Pointer(&cSlice_` + f.RosName + `)), m.` + f.GoName + `[:])`

	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName == "" {
		// Primitive value Slice
		m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
		return `primitives.` + ucFirst(f.RosType) + `__Sequence_into_C((*primitives.C` + ucFirst(f.RosType) + `__Sequence)(unsafe.Pointer(&mem.` + f.CName + `)), m.` + f.GoName + `)`

	} else if f.TypeArray == "" && f.PkgName == "" {
		// Primitive value single

		// string and U16String are special cases because they have custom
		// serialization implementations but still use a non-generated type in
		// generated message fields.
		if f.RosType == "string" {
			m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
			return "primitives.StringIntoC(unsafe.Pointer(&mem." + f.CName + "), m." + f.GoName + ")"
		} else if f.RosType == "U16String" {
			m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
			return "primitives.U16StringIntoC(unsafe.Pointer(&mem." + f.CName + "), m." + f.GoName + ")"
		}
		return `mem.` + f.CName + ` = C.` + f.CType + `(m.` + f.GoName + `)`
	}
	return "//<MISSING cSerializationCode!!>"
}

func (p *parser) goSerializationCode(f *ROS2Field, m *ROS2Message) string {

	if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName != "" && f.PkgIsLocal {
		// Complex value Array local package reference
		return ucFirst(f.RosType) + `__Array_to_Go(m.` + f.GoName + `[:], mem.` + f.CName + `[:])`

	} else if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName != "" {
		// Complex value Array remote package reference
		return `cSlice_` + f.RosName + ` := mem.` + f.CName + `[:]
	` + f.GoPkgReference() + ucFirst(f.RosType) + `__Array_to_Go(m.` + f.GoName + `[:], *(*[]` + f.GoPkgReference() + `C` + ucFirst(f.RosType) + `)(unsafe.Pointer(&cSlice_` + f.RosName + `)))`

	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName != "" && f.PkgIsLocal {
		// Complex value Slice local package reference
		return ucFirst(f.RosType) + `__Sequence_to_Go(&m.` + f.GoName + `, mem.` + f.CName + `)`

	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName != "" && !f.PkgIsLocal {
		// Complex value Slice remote package reference
		return f.GoPkgReference() + ucFirst(f.RosType) + `__Sequence_to_Go(&m.` + f.GoName + `, *(*` + f.GoPkgReference() + `C` + ucFirst(f.RosType) + `__Sequence)(unsafe.Pointer(&mem.` + f.CName + `)))`

	} else if f.TypeArray == "" && f.PkgName != "" {
		// Complex value single
		return f.GoPkgReference() + f.GoType + "TypeSupport.AsGoStruct(&m." + f.GoName + ", unsafe.Pointer(&mem." + f.CName + "))"

	} else if f.TypeArray != "" && f.ArraySize > 0 && f.PkgName == "" {
		// Primitive value Array
		m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
		return `cSlice_` + f.RosName + ` := mem.` + f.CName + `[:]
	` + `primitives.` + ucFirst(f.RosType) + `__Array_to_Go(m.` + f.GoName + `[:], *(*[]primitives.C` + ucFirst(f.RosType) + `)(unsafe.Pointer(&cSlice_` + f.RosName + `)))`

	} else if f.TypeArray != "" && f.ArraySize == 0 && f.PkgName == "" {
		// Primitive value Slice
		m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
		return `primitives.` + ucFirst(f.RosType) + `__Sequence_to_Go(&m.` + f.GoName + `, *(*primitives.C` + ucFirst(f.RosType) + `__Sequence)(unsafe.Pointer(&mem.` + f.CName + `)))`

	} else if f.TypeArray == "" && f.PkgName == "" {
		// Primitive value single

		// string and U16String are special cases because they have custom
		// serialization implementations but still use a non-generated type in
		// generated message fields.
		if f.RosType == "string" {
			m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
			return "primitives.StringAsGoStruct(&m." + f.GoName + ", unsafe.Pointer(&mem." + f.CName + "))"
		} else if f.RosType == "U16String" {
			m.GoImports[p.config.RclgoImportPath+"/pkg/rclgo/primitives"] = "primitives"
			return "primitives.U16StringAsGoStruct(&m." + f.GoName + ", unsafe.Pointer(&mem." + f.CName + "))"
		}
		return `m.` + f.GoName + ` = ` + f.GoType + `(mem.` + f.CName + `)`

	}
	return "//<MISSING goSerializationCode!!>"
}

func defaultCode(f *ROS2Field) string {
	if f.PkgName != "" && f.TypeArray != "" && f.DefaultValue == "" {
		// Complex value array and slice common default
		if f.ArraySize == 0 {
			return "t." + f.GoName + " = nil"
		}
		return "for i := range t." + f.GoName + " {\n" +
			"\t\tt." + f.GoName + "[i].SetDefaults()\n" +
			"\t}"

	} else if f.PkgName != "" && f.TypeArray != "" {
		defaultValues := splitMsgDefaultArrayValues(f.RosType, f.DefaultValue)
		// Complex value array and slice
		sb := strings.Builder{}
		if f.ArraySize <= 0 && len(defaultValues) > 0 { // Init a slice
			fmt.Fprint(&sb, `t.`, f.GoName, ` = make(`, f.TypeArray, f.GoPkgReference(), f.GoType, `, `, len(defaultValues), ")\n\t")
		}

		fmt.Fprint(&sb,
			"for i := range t.", f.GoName, " {\n",
			"\t\tt.", f.GoName, "[i].SetDefaults()\n",
			"\t}",
		)
		return sb.String()

	} else if f.PkgName != "" && f.TypeArray == "" {
		// Complex value single
		return `t.` + f.GoName + ".SetDefaults()"

	} else if f.DefaultValue != "" && f.TypeArray != "" {
		// Primitive value array and slice
		defaultValues := splitMsgDefaultArrayValues(f.RosType, f.DefaultValue)
		for i := range defaultValues {
			defaultValues[i] = defaultValueSanitizer(f.RosType, defaultValues[i])
		}
		return `t.` + f.GoName + ` = ` + f.TypeArray + f.GoPkgReference() + f.GoType + `{` + strings.Join(defaultValues, ",") + `}`

	} else if f.DefaultValue == "" && f.TypeArray != "" {
		// Primitive value array and slice common default
		if f.ArraySize == 0 {
			return "t." + f.GoName + " = nil"
		}
		return fmt.Sprint("t.", f.GoName, " = [", f.ArraySize, "]", f.GoType, "{}")

	} else if f.DefaultValue != "" {
		// Primitive value single
		return `t.` + f.GoName + ` = ` + defaultValueSanitizer(f.RosType, f.DefaultValue)

	} else if f.DefaultValue == "" {
		// Primitive value single common default
		return "t." + f.GoName + " = " + primitiveCommonDefault(f)
	}
	return "//<MISSING defaultCode!!>"
}

func primitiveCommonDefault(f *ROS2Field) string {
	switch f.RosType {
	case "string", "wstring", "U16String":
		return `""`
	case "bool":
		return "false"
	case "float32", "float64", "byte", "char", "int8", "int16",
		"int32", "int64", "uint8", "uint16", "uint32", "uint64":
		return "0"
	default:
		panic("common default value for ROS type " + f.RosType + " is not defined")
	}
}

func cloneCode(f *ROS2Field) string {
	if f.PkgName != "" && f.TypeArray != "" && f.ArraySize == 0 {
		// complex value slice
		return "if t." + f.GoName + " != nil {\n" +
			"\t\tc." + f.GoName + " = make([]" + f.GoPkgReference() + f.GoType + ", len(t." + f.GoName + "))\n" +
			"\t\t" + f.GoPkgReference() + "Clone" + f.GoType + "Slice(c." + f.GoName + ", t." + f.GoName + ")\n" +
			"\t}"
	} else if f.PkgName != "" && f.TypeArray != "" && f.ArraySize > 0 {
		// complex value array
		return f.GoPkgReference() + "Clone" + f.GoType + "Slice(c." + f.GoName + "[:], t." + f.GoName + "[:])"
	} else if f.PkgName != "" && f.TypeArray == "" {
		// complex value single
		return "c." + f.GoName + " = *t." + f.GoName + ".Clone()"
	} else if f.PkgName == "" && f.TypeArray != "" && f.ArraySize == 0 {
		// primitive value slice
		return "if t." + f.GoName + " != nil {\n" +
			"\t\tc." + f.GoName + " = make([]" + f.GoType + ", len(t." + f.GoName + "))\n" +
			"\t\tcopy(c." + f.GoName + ", t." + f.GoName + ")\n" +
			"\t}"
	} else if f.PkgName == "" {
		// primitive value single and array
		return "c." + f.GoName + " = t." + f.GoName
	}
	return "//<MISSING cloneCode!!>"
}

// checkBoundsCode returns the CheckBounds statements for f: a length check
// for a bounded sequence, and a check of each nested message. Nested
// messages are checked through types.CheckBounds, so that messages generated
// without CheckBounds still compile.
func checkBoundsCode(f *ROS2Field) string {
	var b strings.Builder
	if bound := strings.TrimPrefix(f.ArrayBounded, "<="); bound != "" {
		fmt.Fprintf(&b, "if len(t.%s) > %s {\n\t\treturn types.BoundExceeded(%q, len(t.%s), %s)\n\t}", f.GoName, bound, f.RosName, f.GoName, bound)
	}
	if f.PkgName == "" {
		return b.String()
	}
	if b.Len() > 0 {
		b.WriteString("\n\t")
	}
	if f.TypeArray == "" {
		fmt.Fprintf(&b, "if err := types.CheckBounds(&t.%s); err != nil {\n\t\treturn types.InField(%q, err)\n\t}", f.GoName, f.RosName)
	} else {
		fmt.Fprintf(&b, "for i := range t.%s {\n\t\tif err := types.CheckBounds(&t.%s[i]); err != nil {\n\t\t\treturn types.InElement(%q, i, err)\n\t\t}\n\t}", f.GoName, f.GoName, f.RosName)
	}
	return b.String()
}

// Names and values are checked as rosidl checks them (rosidl_adapter's
// parser), so that a definition rosidl accepts is never rejected here, and a
// line rosidl would reject is reported instead of being turned into a field.
var (
	validLowerName    = regexp.MustCompile(`^[a-z][a-z0-9_]*$`)
	validConstantName = regexp.MustCompile(`^[A-Z](?:[A-Z0-9_]?[A-Z0-9]+)*$`)
	validTypeName     = regexp.MustCompile(`^[A-Z][A-Za-z0-9]*$`)
)

// isValidLowerName reports whether s is a valid ROS package or field name.
func isValidLowerName(s string) bool {
	return validLowerName.MatchString(s) && !strings.Contains(s, "__") && !strings.HasSuffix(s, "_")
}

func checkField(capture map[string]string) error {
	pkg, typ, name, def := capture["package"], capture["type"], capture["field"], capture["default"]
	_, primitive := primitiveTypeMappings[typ]
	switch {
	case pkg != "" && !isValidLowerName(pkg):
		return fmt.Errorf("invalid package name '%s'", pkg)
	case pkg != "" && !validTypeName.MatchString(typ):
		return fmt.Errorf("invalid message type name '%s/%s'", pkg, typ)
	case pkg == "" && !primitive && !validTypeName.MatchString(typ):
		return fmt.Errorf("unknown type '%s': not a primitive type or a valid message type name", typ)
	case !isValidLowerName(name):
		return fmt.Errorf("invalid field name '%s'", name)
	case def == "":
		return nil
	case pkg != "" || !primitive:
		return fmt.Errorf("unexpected '%s' after field '%s': only fields of primitive types have default values", def, name)
	}
	stringBound := 0
	if b := strings.TrimPrefix(capture["boundedString"], "<="); b != "" {
		stringBound, _ = strconv.Atoi(b)
	}
	if capture["array"] == "" {
		return checkPrimitiveValue(typ, def, stringBound)
	}
	if !strings.HasPrefix(def, "[") || !strings.HasSuffix(def, "]") {
		return fmt.Errorf("default value of array field '%s' must be enclosed in [ ]: '%s'", name, def)
	}
	values := splitMsgDefaultArrayValues("", def)
	if strings.TrimSpace(normalizeMsgDefaultArrayValue(def)) == "" {
		values = nil
	}
	size, _ := strconv.Atoi(capture["size"])
	switch {
	case capture["bounded"] != "" && len(values) > size:
		return fmt.Errorf("default value of field '%s' has %d elements, more than its bound %d", name, len(values), size)
	case capture["bounded"] == "" && capture["size"] != "" && len(values) != size:
		return fmt.Errorf("default value of field '%s' has %d elements, its array size is %d", name, len(values), size)
	}
	for _, v := range values {
		if err := checkPrimitiveValue(typ, strings.TrimSpace(v), stringBound); err != nil {
			return err
		}
	}
	return nil
}

// checkPrimitiveValue reports whether value is a valid value of the primitive
// ROS type, following rosidl_adapter's parse_primitive_value_string. A
// stringBound > 0 limits the length of string values.
func checkPrimitiveValue(rosType, value string, stringBound int) error {
	invalid := func(why string) error {
		return fmt.Errorf("invalid %s value '%s': %s", rosType, value, why)
	}
	parseInt := func(bits int, unsigned bool) error {
		var err error
		if unsigned {
			if _, err = strconv.ParseUint(value, 10, bits); err != nil {
				_, err = strconv.ParseUint(value, 0, bits)
			}
		} else {
			if _, err = strconv.ParseInt(value, 10, bits); err != nil {
				_, err = strconv.ParseInt(value, 0, bits)
			}
		}
		if err != nil {
			return invalid(fmt.Sprintf("must be a %d-bit integer", bits))
		}
		return nil
	}
	switch rosType {
	case "bool":
		switch strings.ToLower(value) {
		case "true", "false", "1", "0":
			return nil
		}
		return invalid("must be true, false, 1 or 0")
	case "byte", "char", "uint8":
		return parseInt(8, true)
	case "uint16", "uint32", "uint64":
		bits, _ := strconv.Atoi(rosType[4:])
		return parseInt(bits, true)
	case "int8", "int16", "int32", "int64":
		bits, _ := strconv.Atoi(rosType[3:])
		return parseInt(bits, false)
	case "float32", "float64":
		f, err := strconv.ParseFloat(value, 64)
		if err != nil {
			return invalid("must be a floating point number")
		}
		if math.IsInf(f, 0) || math.IsNaN(f) {
			// rosidl's .msg parser accepts them, but its C and C++ generators
			// fail on them, so no built package has them.
			return invalid("infinities and NaN are not supported by the rosidl generators")
		}
		return nil
	case "string", "wstring":
		for _, q := range []string{`"`, `'`} {
			if len(value) >= 2 && strings.HasPrefix(value, q) && strings.HasSuffix(value, q) {
				value = value[1 : len(value)-1]
				for i := 0; i < len(value); i++ {
					if value[i:i+1] == q && (i == 0 || value[i-1] != '\\') {
						return invalid("inner quotes must be escaped")
					}
				}
				value = strings.ReplaceAll(value, `\`+q, q)
				break
			}
		}
		if stringBound > 0 && utf8.RuneCountInString(value) > stringBound {
			return invalid(fmt.Sprintf("longer than the bound of %d characters", stringBound))
		}
		return nil
	}
	return nil
}

// generatedMethods are the methods generated on message types; a field may not
// have one of these Go names.
var generatedMethods = map[string]bool{
	"CallForEach": true, "Clone": true, "CloneMsg": true, "GetGoalAccepted": true,
	"GetGoalDescription": true, "GetGoalID": true, "GetTypeSupport": true,
	"SetDefaults": true, "CheckBounds": true, "SetGoalDescription": true, "SetGoalID": true,
}

// uniqueFieldGoName returns name, with underscores appended (as rclpy appends
// one to Python keywords) if it is the name of a generated method or of an
// earlier field of msg: a_1 and a1 both become A1 in Go.
func uniqueFieldGoName(name string, msg *ROS2Message) string {
	taken := func(n string) bool {
		if generatedMethods[n] {
			return true
		}
		for _, f := range msg.Fields {
			if f.GoName == n {
				return true
			}
		}
		return false
	}
	for taken(name) {
		name += "_"
	}
	return name
}
