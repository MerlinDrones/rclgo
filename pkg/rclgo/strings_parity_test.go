package rclgo_test

import (
	"bufio"
	"encoding/hex"
	"encoding/json"
	"os/exec"
	"strings"
	"testing"

	test_msgs "github.com/merlindrones/rclgo/pkg/msgs/test_msgs/msg"
	"github.com/merlindrones/rclgo/pkg/rclgo"
	"github.com/merlindrones/rclgo/pkg/rclgo/internal/rclgotest"
	"github.com/merlindrones/rclgo/pkg/rclgo/types"
	"github.com/stretchr/testify/require"
)

// Strings and wstrings with embedded NULs, invalid UTF-8, characters outside
// the BMP and 100 000 characters convert as rclpy converts them, in both
// directions, offline (CDR bytes, no middleware). What is pinned:
//   - a string ends at an embedded NUL (CDR strings are NUL-terminated), on
//     both sides; a wstring keeps it;
//   - invalid UTF-8 passes through a string unchanged in Go (rclpy reads it
//     as U+FFFD), and becomes U+FFFD in a wstring on both sides;
//   - rclpy refuses to send a lone surrogate (Go strings can't hold one).
func TestStringEdgeCasesMatchRclpy(t *testing.T) {
	rclgotest.RequireROS2(t)
	if out, err := exec.Command("python3", "-P", "-c", "import rclpy, test_msgs").CombinedOutput(); err != nil {
		rclgotest.Unavailable(t, "rclpy or test_msgs not importable: %s", out)
	}
	cmd := exec.Command("python3", "-P", "testdata/interop/strings_codec.py")
	stdin, err := cmd.StdinPipe()
	require.NoError(t, err)
	stdout, err := cmd.StdoutPipe()
	require.NoError(t, err)
	require.NoError(t, cmd.Start())
	t.Cleanup(func() { stdin.Close(); cmd.Wait() }) //nolint:errcheck
	replies := bufio.NewScanner(stdout)
	replies.Buffer(make([]byte, 1<<20), 64<<20)
	ask := func(req map[string]any) map[string]any {
		t.Helper()
		b, err := json.Marshal(req)
		require.NoError(t, err)
		_, err = stdin.Write(append(b, '\n'))
		require.NoError(t, err)
		require.True(t, replies.Scan(), "no reply from strings_codec.py: %v", replies.Err())
		var r map[string]any
		require.NoError(t, json.Unmarshal(replies.Bytes(), &r))
		return r
	}
	codePoints := func(s string) []int {
		var cps []int
		for _, r := range s {
			cps = append(cps, int(r))
		}
		return cps
	}
	fromCodePoints := func(v any) string {
		var b strings.Builder
		for _, c := range v.([]any) {
			b.WriteRune(rune(int(c.(float64))))
		}
		return b.String()
	}
	message := func(typ, s string) types.Message {
		if typ == "Strings" {
			return &test_msgs.Strings{StringValue: s}
		}
		return &test_msgs.WStrings{WstringValue: s}
	}
	field := func(m types.Message) string {
		if s, ok := m.(*test_msgs.Strings); ok {
			return s.StringValue
		}
		return m.(*test_msgs.WStrings).WstringValue
	}
	long := strings.Repeat("x", 100000)

	// rclgo serializes; rclgo and rclpy read the bytes back.
	for _, tc := range []struct {
		typ, in, goReads, pyReads string
	}{
		{"Strings", "a\x00b", "a", "a"},
		{"WStrings", "a\x00b", "a\x00b", "a\x00b"},
		{"Strings", "\xff\xfe", "\xff\xfe", "��"},
		{"WStrings", "\xff\xfe", "��", "��"},
		{"Strings", "😀 é", "😀 é", "😀 é"},
		{"WStrings", "😀 é", "😀 é", "😀 é"},
		{"Strings", long, long, long},
		{"WStrings", long, long, long},
	} {
		msg := message(tc.typ, tc.in)
		buf, err := rclgo.Serialize(msg)
		require.NoError(t, err)
		back, err := rclgo.Deserialize(buf, msg.GetTypeSupport())
		require.NoError(t, err)
		require.Equal(t, tc.goReads, field(back), "%s %q read back by rclgo", tc.typ, tc.in[:min(len(tc.in), 12)])
		r := ask(map[string]any{"op": "deserialize", "type": tc.typ, "hex": hex.EncodeToString(buf)})
		require.Nil(t, r["error"], "rclpy reading %s %q", tc.typ, tc.in[:min(len(tc.in), 12)])
		require.Equal(t, codePoints(tc.pyReads), codePoints(fromCodePoints(r["value"])),
			"%s %q read by rclpy", tc.typ, tc.in[:min(len(tc.in), 12)])
	}

	// rclpy serializes; rclgo reads.
	for _, tc := range []struct {
		typ     string
		in      []int
		goReads string
	}{
		{"Strings", []int{'a', 0, 'b'}, "a"},
		{"WStrings", []int{'a', 0, 'b'}, "a\x00b"},
		{"Strings", []int{0x1f600}, "😀"},
		{"WStrings", []int{0x1f600}, "😀"},
	} {
		r := ask(map[string]any{"op": "serialize", "type": tc.typ, "value": tc.in})
		require.Nil(t, r["error"], "rclpy serializing %s %v", tc.typ, tc.in)
		buf, err := hex.DecodeString(r["hex"].(string))
		require.NoError(t, err)
		ts := message(tc.typ, "").GetTypeSupport()
		got, err := rclgo.Deserialize(buf, ts)
		require.NoError(t, err)
		require.Equal(t, tc.goReads, field(got), "%s %v from rclpy", tc.typ, tc.in)
	}
	for _, typ := range []string{"Strings", "WStrings"} {
		r := ask(map[string]any{"op": "serialize", "type": typ, "value": []int{0xd800}})
		require.Contains(t, r["error"], "surrogates not allowed", "rclpy sending a lone surrogate in %s", typ)
	}
}
