package params

import (
	"os"
	"path/filepath"
	"testing"
)

// LoadYAML must return an error, not panic, on any file contents.
func FuzzLoadYAML(f *testing.F) {
	f.Add("ros__parameters:\n  a: 1\n  b: [1, 2]\n  c: {d: x}\n")
	f.Add("node:\n  ros__parameters:\n    x: [true, 1, \"s\"]\n")
	f.Add("ros__parameters: 5\n")
	f.Add("ros__parameters:\n  deep: {a: {b: {c: [[1]]}}}\n")
	dir := f.TempDir()
	f.Fuzz(func(t *testing.T, contents string) {
		path := filepath.Join(dir, "p.yaml")
		if err := os.WriteFile(path, []byte(contents), 0o600); err != nil {
			t.Fatal(err)
		}
		m := newBareManager()
		_ = LoadYAML(m, "node", path)
	})
}

// The RCL_PARAM_* value parser must not panic on any string, with or without
// an existing parameter to take the type from.
func FuzzParseEnvVarValue(f *testing.F) {
	for _, s := range []string{"1", "1.5", "true", "[1,2]", "[a, b]", "[", "]", "[1, x]", "", "0x10", "1e309"} {
		f.Add(s)
	}
	existing := []*Parameter{
		nil,
		{Name: "i", Value: Value{Kind: KindInt64}},
		{Name: "d", Value: Value{Kind: KindDoubleArray}},
		{Name: "s", Value: Value{Kind: KindStringArray}},
		{Name: "b", Value: Value{Kind: KindBoolArray}},
	}
	f.Fuzz(func(t *testing.T, s string) {
		for _, e := range existing {
			_, _ = parseEnvVarValue(s, e)
		}
	})
}
