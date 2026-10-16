package gogen

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// Interface definitions are checked as rosidl checks them: what rosidl
// accepts is accepted, and what it rejects is reported, never turned into a
// field.

func TestParserAcceptsValidDefinitions(t *testing.T) {
	for _, line := range []string{
		"int32 a",
		"int32 a 5",
		"int32 a -2147483648",
		"uint64 a 18446744073709551615",
		"int32 a 0x10",
		"uint8 a 255",
		"byte a 0",
		"char a 255",
		"bool a True",
		"bool a 0",
		"float64 a 1.5e3",
		"float32 a -0.5",
		`string a "with spaces"`,
		`string a 'single'`,
		`string a "escaped \" quote"`,
		"string a unquoted",
		`string<=5 a "five5"`,
		"int32[3] a [1, 2, 3]",
		"int32[<=3] a [1, 2]",
		"int32[] a []",
		`string[] a ["a", "b,c"]`,
		`string<=2[<=2] a ["ab", "c"]`,
		"geometry_msgs/Point p",
		"geometry_msgs/Point[<=2] p",
		"Inner inner",
		"int32 x_y_z_2",
		"int32 a5_b # comment",
		"int32 a 5 # comment with = and #",
		"float32 v_ref                      # ADC channel voltage reference (lsb=scale/resolution)",
		"int32 CONST = 5",
		"int32 CONST_2 = -5 # comment",
		`string C_STR = "string, with comma"`,
		"bool C_B = true",
		"uint8 C_U8 = 255",
	} {
		p := parser{config: &DefaultConfig}
		if _, err := p.parseMessageLine(line, ROS2MessageNew("pkg", "Msg")); err != nil {
			t.Errorf("%q: %v", line, err)
		}
	}
}

func TestParserRejectsInvalidDefinitions(t *testing.T) {
	for line, want := range map[string]string{
		"this is not a valid line !!":    "unknown type 'this'",
		"int32 a abc":                    "invalid int32 value 'abc'",
		"int32 a 1 2":                    "invalid int32 value '1 2'",
		"int8 a 128":                     "invalid int8 value '128'",
		"uint8 a -1":                     "invalid uint8 value '-1'",
		"byte a 256":                     "invalid byte value '256'",
		"bool a yes":                     "invalid bool value 'yes'",
		"float64 a 1,5":                  "invalid float64 value '1,5'",
		`string a "inner " quote"`:       "inner quotes must be escaped",
		`string<=3 a "four"`:             "longer than the bound of 3",
		"int32[3] a [1, 2]":              "has 2 elements, its array size is 3",
		"int32[<=1] a [1, 2]":            "more than its bound 1",
		"int32[] a 5":                    "must be enclosed in [ ]",
		"int32[] a [1, x]":               "invalid int32 value 'x'",
		"NoSuchPkg/Thing ref":            "invalid package name 'NoSuchPkg'",
		"no__pkg/Thing ref":              "invalid package name",
		"geometry_msgs/point ref":        "invalid message type name",
		"geometry_msgs/Point p 5":        "only fields of primitive types have default values",
		"Inner inner [1]":                "only fields of primitive types have default values",
		"int32 Bad":                      "invalid field name 'Bad'",
		"int32 bad_":                     "invalid field name 'bad_'",
		"int32 bad__name":                "invalid field name 'bad__name'",
		"int32 a!!":                      "couldn't parse",
		"int32 bad_const = 1":            "invalid constant name 'bad_const'",
		"int32 BAD_ = 1":                 "invalid constant name 'BAD_'",
		"int32 C = 3000000000":           "invalid int32 value '3000000000'",
		"int32[2] C = [1, 2]":            "primitive, non-array type",
		"geometry_msgs/Point C = 1":      "primitive, non-array type",
		"Point C = 1":                    "Unknown ROS2 Constant type 'Point'",
		"bool C = maybe":                 "invalid bool value 'maybe'",
		"string<=5[<=1] a [\"toolong\"]": "longer than the bound of 5",
	} {
		p := parser{config: &DefaultConfig}
		obj, err := p.parseMessageLine(line, ROS2MessageNew("pkg", "Msg"))
		if err == nil {
			t.Errorf("%q: accepted as %#v", line, obj)
		} else if !strings.Contains(err.Error(), want) {
			t.Errorf("%q: error %q does not contain %q", line, err, want)
		}
	}
}

func TestParserReportsEveryBadLine(t *testing.T) {
	p := parser{config: &DefaultConfig}
	err := p.ParseROS2Message(ROS2MessageNew("pkg", "Msg"), "int32 ok\nint32 a abc\n\n# comment\nbad line here\n")
	if err == nil {
		t.Fatal("no error")
	}
	for _, want := range []string{"line 2", "line 5"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error %q does not mention %s", err, want)
		}
	}

	// A third section in a service used to index past the sections.
	srv := NewROS2Service("pkg", "Srv")
	if err := p.ParseService(srv, "int32 a\n---\nint32 b\n---\nint32 c\n"); err == nil ||
		!strings.Contains(err.Error(), "too many sections") {
		t.Errorf("service with three sections: %v", err)
	}
}

func writeInterface(t *testing.T, root, pkg, rel, content string) {
	t.Helper()
	path := filepath.Join(root, "share", pkg, rel)
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(content), 0o644); err != nil {
		t.Fatal(err)
	}
}

// Errors in a package's definitions are returned, naming the file and the
// problem, rather than only printed; a field whose type does not exist is
// reported instead of producing code that does not compile.
func TestGeneratorReturnsDefinitionErrors(t *testing.T) {
	root := t.TempDir()
	writeInterface(t, root, "bad_msgs", "msg/Syntax.msg", "int32 ok\nthis is not valid\n")
	writeInterface(t, root, "bad_msgs", "msg/Refs.msg",
		"no_such_pkg/Thing a\nbad_msgs/Missing b\nMissing c\nbad_msgs/Fine d\nFine e\n")
	writeInterface(t, root, "bad_msgs", "msg/Fine.msg", "int32 x\n")
	writeInterface(t, root, "bad_msgs", "srv/Svc.srv", "Missing req\n---\nno_such_pkg/Thing res\n")

	config := DefaultConfig
	config.RootPaths = []string{root}
	config.DestPath = t.TempDir()
	config.ROSPkgIncludes = []string{"bad_msgs"}
	config.RegexIncludes = nil
	config.GoPkgIncludes = nil
	err := New(&config).GenerateGolangMessageTypes()
	if err == nil {
		t.Fatal("no error")
	}
	for _, want := range []string{
		"Syntax.msg: error on line 2: invalid field 'this is not valid'",
		"Refs.msg: field 'a' has type no_such_pkg/Thing, but package no_such_pkg is not found",
		"Refs.msg: field 'b' has type bad_msgs/Missing, but package bad_msgs has no message Missing",
		"Refs.msg: field 'c' has type bad_msgs/Missing",
		"Svc.srv: field 'req' has type bad_msgs/Missing",
		"Svc.srv: field 'res' has type no_such_pkg/Thing",
		"no_such_pkg: package not found",
	} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("error lacks %q:\n%v", want, err)
		}
	}
	for _, field := range []string{"'d'", "'e'"} {
		if strings.Contains(err.Error(), "field "+field) {
			t.Errorf("field %s refers to an existing message but is reported:\n%v", field, err)
		}
	}

	config.ROSPkgIncludes = []string{"no_such_package"}
	if err := New(&config).GenerateGolangMessageTypes(); err == nil ||
		!strings.Contains(err.Error(), "no_such_package: package not found") {
		t.Errorf("unknown package in ROSPkgIncludes: %v", err)
	}
}

// Every interface package installed with ROS generates without errors, and
// the generated code compiles.
func TestGenerateAllInstalledPackages(t *testing.T) {
	if testing.Short() {
		t.Skip("builds the bindings of every installed package")
	}
	distro := os.Getenv("ROS_DISTRO")
	root := "/opt/ros/" + distro
	if _, err := os.Stat(filepath.Join(root, "share", "std_msgs")); distro == "" || err != nil {
		t.Skip("needs a sourced ROS 2 installation")
	}
	repo, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	dest := t.TempDir()
	config := DefaultConfig
	config.RootPaths = []string{root}
	config.DestPath = dest
	config.MessageModulePrefix = "example.com/allmsgs"
	config.RclgoImportPath = "github.com/merlindrones/rclgo"
	config.CGOFlagsPath = filepath.Join(dest, "cgo-flags.env")
	config.RegexIncludes = nil
	config.ROSPkgIncludes = nil
	config.GoPkgIncludes = nil
	gen := New(&config)
	if err := gen.GenerateGolangMessageTypes(); err != nil {
		t.Fatalf("GenerateGolangMessageTypes: %v", err)
	}
	if err := gen.GenerateCGOFlags(); err != nil {
		t.Fatalf("GenerateCGOFlags: %v", err)
	}
	gomod := "module example.com/allmsgs\n\ngo 1.22\n\nrequire github.com/merlindrones/rclgo v0.0.0\n\n" +
		"replace github.com/merlindrones/rclgo => " + repo + "\n"
	if err := os.WriteFile(filepath.Join(dest, "go.mod"), []byte(gomod), 0o644); err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command("bash", "-c", "set -e; source cgo-flags.env; go mod tidy; go build ./...")
	cmd.Dir = dest
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("building the generated bindings: %v\n%s", err, out)
	}
}

func TestParseMessageAndRules(t *testing.T) {
	msg, err := ParseMessage(&DefaultConfig, "# comment\nint32 a 5\nstring B = \"b\"\n")
	if err != nil {
		t.Fatal(err)
	}
	if len(msg.Fields) != 1 || msg.Fields[0].RosName != "a" || len(msg.Constants) != 1 || msg.Constants[0].RosName != "B" {
		t.Errorf("parsed %+v %+v", msg.Fields, msg.Constants)
	}
	if _, err := ParseMessage(&DefaultConfig, "not valid at all"); err == nil {
		t.Error("ParseMessage accepted an invalid definition")
	}

	rule, err := NewRule("std_.*")
	if err != nil {
		t.Fatal(err)
	}
	set := RuleSet{rule}
	for pkg, want := range map[string]bool{"std_msgs": true, "std_srvs": true, "my_std_msgs": false, "": false} {
		if set.Includes(pkg) != want {
			t.Errorf("Includes(%q) = %v", pkg, !want)
		}
	}
	if _, err := NewRule("("); err == nil {
		t.Error("NewRule accepted an invalid pattern")
	}
}
