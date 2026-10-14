/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo_test

import (
	"fmt"
	"math"
	"reflect"
	"testing"
	"unicode/utf8"
)

// Go -> C -> Go conversion of every test_msgs type, with field values taken
// from the fuzz input. Each run converts two different messages into the same
// C buffer, so the second conversion frees what the first one allocated, and
// the C message must then read back as the second message. Freed or
// uninitialized memory is easiest to catch with glibc filling it with junk:
//
//	MALLOC_PERTURB_=165 go test -run '^$' -fuzz FuzzConversionRoundTrip ./pkg/rclgo/
func FuzzConversionRoundTrip(f *testing.F) {
	f.Add([]byte{})
	f.Add([]byte("seed: a few bytes \xe2\x82\xac \xff\x00\x01\x02\x03"))
	f.Add([]byte{3, 3, 3, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0x7f, 0x80, 1})
	f.Fuzz(func(t *testing.T, data []byte) {
		for _, tc := range testMessageTypes {
			src := &fuzzSource{data: data}
			first, second := tc.ts.New(), tc.ts.New()
			fuzzFill(reflect.ValueOf(first).Elem(), src)
			fuzzFill(reflect.ValueOf(second).Elem(), src)

			buf := tc.ts.PrepareMemory()
			tc.ts.AsCStruct(buf, first)
			tc.ts.AsCStruct(buf, second)
			out := tc.ts.New()
			tc.ts.AsGoStruct(out, buf)
			tc.ts.ReleaseMemory(buf)
			// %#v tells NaN from other values and -0 from 0.
			if got, want := fmt.Sprintf("%#v", out), fmt.Sprintf("%#v", second); got != want {
				t.Fatalf("%s after converting twice:\n got %s\nwant %s", tc.name, got, want)
			}
		}
	})
}

// fuzzSource hands out the fuzz input byte by byte, then zeros.
type fuzzSource struct {
	data []byte
	pos  int
}

func (s *fuzzSource) byte() byte {
	if s.pos >= len(s.data) {
		return 0
	}
	b := s.data[s.pos]
	s.pos++
	return b
}

func (s *fuzzSource) uint64() uint64 {
	var v uint64
	for i := 0; i < 8; i++ {
		v = v<<8 | uint64(s.byte())
	}
	return v
}

// fuzzFill sets every exported field of v from src. Sequences get 0 to 3
// elements (the bound of every bounded sequence in test_msgs) and strings up
// to 8 runes of valid UTF-8, which wstrings convert without loss.
func fuzzFill(v reflect.Value, src *fuzzSource) {
	switch v.Kind() {
	case reflect.Struct:
		for i := 0; i < v.NumField(); i++ {
			if v.Type().Field(i).IsExported() {
				fuzzFill(v.Field(i), src)
			}
		}
	case reflect.Array:
		for i := 0; i < v.Len(); i++ {
			fuzzFill(v.Index(i), src)
		}
	case reflect.Slice:
		n := int(src.byte() % 4)
		if n == 0 {
			v.Set(reflect.Zero(v.Type())) // an empty C sequence reads back as nil
			return
		}
		s := reflect.MakeSlice(v.Type(), n, n)
		for i := 0; i < n; i++ {
			fuzzFill(s.Index(i), src)
		}
		v.Set(s)
	case reflect.Bool:
		v.SetBool(src.byte()&1 == 1)
	case reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		v.SetInt(int64(src.uint64()))
	case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		v.SetUint(src.uint64())
	case reflect.Float32:
		v.SetFloat(float64(math.Float32frombits(uint32(src.uint64()))))
	case reflect.Float64:
		v.SetFloat(math.Float64frombits(src.uint64()))
	case reflect.String:
		n := int(src.byte() % 9)
		runes := make([]rune, 0, n)
		for i := 0; i < n; i++ {
			r := rune(src.byte())<<8 | rune(src.byte())
			if r == 0 || !utf8.ValidRune(r) {
				r = 'x' // C strings end at NUL; surrogates aren't valid UTF-8
			}
			runes = append(runes, r)
		}
		v.SetString(string(runes))
	default:
		panic("unhandled field kind " + v.Kind().String())
	}
}
