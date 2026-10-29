/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package types

import (
	"errors"
	"fmt"
)

// ErrBoundExceeded is returned, wrapped, when a message to be sent holds a
// bounded sequence (a field like int32[<=3]) with more elements than its
// bound. rclcpp refuses such a sequence when it is assigned; rmw
// implementations differ (Fast DDS and Zenoh fail to publish it, Cyclone DDS
// sends it), so rclgo checks before converting a message. Bounded strings are
// not checked, as in rclcpp.
var ErrBoundExceeded = errors.New("bounded sequence exceeds its bound")

// BoundsChecker is implemented by generated messages. CheckBounds returns an
// error wrapping ErrBoundExceeded if a bounded sequence in the message, or in
// a message nested in it, is longer than its bound.
type BoundsChecker interface {
	CheckBounds() error
}

// CheckBounds checks msg's bounded sequences if msg implements BoundsChecker
// (messages generated before rclgo-gen added the method don't).
func CheckBounds(msg any) error {
	if c, ok := msg.(BoundsChecker); ok {
		return c.CheckBounds()
	}
	return nil
}

// BoundExceeded returns the error for the field named field with n elements
// and the given bound. Generated code calls it.
func BoundExceeded(field string, n, bound int) error {
	return &boundError{path: field, n: n, bound: bound}
}

// InField returns err, from a message nested in the field named field, with
// the field prepended to its path. Generated code calls it.
func InField(field string, err error) error {
	var b *boundError
	if errors.As(err, &b) {
		return &boundError{path: field + "." + b.path, n: b.n, bound: b.bound}
	}
	return fmt.Errorf("%s: %w", field, err)
}

// InElement is InField for element i of an array or sequence field.
func InElement(field string, i int, err error) error {
	return InField(fmt.Sprintf("%s[%d]", field, i), err)
}

type boundError struct {
	path     string
	n, bound int
}

func (e *boundError) Error() string {
	return fmt.Sprintf("%s has %d elements, more than its bound of %d", e.path, e.n, e.bound)
}

func (e *boundError) Unwrap() error { return ErrBoundExceeded }
