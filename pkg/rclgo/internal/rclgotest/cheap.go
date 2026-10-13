package rclgotest

/*
#include <malloc.h>

static size_t rclgotest_heap_in_use(void) {
    struct mallinfo2 m = mallinfo2();
    return m.uordblks;
}
*/
import "C"

// CHeapInUse returns the bytes glibc's malloc has handed out and not yet had
// freed (mallinfo2's uordblks). Go's own heap doesn't use malloc, so growth
// across a loop of cgo conversions shows C allocations that are never freed.
func CHeapInUse() uint64 {
	return uint64(C.rclgotest_heap_in_use())
}
