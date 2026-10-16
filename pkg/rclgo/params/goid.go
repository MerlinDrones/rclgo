package params

import (
	"bytes"
	"runtime"
	"strconv"
)

// goroutineID returns the ID of the calling goroutine, from its stack header
// ("goroutine 18 [running]:"). It is only used to recognize a call made from
// inside an on-set callback.
func goroutineID() int64 {
	var buf [64]byte
	b := buf[:runtime.Stack(buf[:], false)]
	b = bytes.TrimPrefix(b, []byte("goroutine "))
	if i := bytes.IndexByte(b, ' '); i >= 0 {
		b = b[:i]
	}
	id, _ := strconv.ParseInt(string(b), 10, 64)
	return id
}
