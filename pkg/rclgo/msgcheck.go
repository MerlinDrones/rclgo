/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

package rclgo

import (
	"errors"
	"fmt"
	"reflect"

	"github.com/merlindrones/rclgo/pkg/rclgo/types"
)

// ErrInvalidMessage is returned, wrapped, when a message passed to rclgo is nil
// or not of the type its publisher, subscription, client, service or action
// was created for. Generated conversion code would otherwise panic on it.
var ErrInvalidMessage = errors.New("invalid message")

// messageType returns the Go type of ts's messages, or nil for a type support
// that has no Go messages (one loaded dynamically, which is serialized-only).
func messageType(ts types.MessageTypeSupport) reflect.Type {
	if _, ok := ts.(*dynamicMessageTypeSupport); ok {
		return nil
	}
	return reflect.TypeOf(ts.New())
}

// checkMessage returns an error wrapping ErrInvalidMessage unless m is a
// non-nil message of type want. A nil want means the type support has no Go
// messages.
func checkMessage(what string, want reflect.Type, m types.Message) error {
	if want == nil {
		return fmt.Errorf("%s: %w: the type support has no Go messages, use the serialized API", what, ErrInvalidMessage)
	}
	if isNilMessage(m) {
		return fmt.Errorf("%s: %w: nil message", what, ErrInvalidMessage)
	}
	if got := reflect.TypeOf(m); got != want {
		return fmt.Errorf("%s: %w: got %v, want %v", what, ErrInvalidMessage, got, want)
	}
	return nil
}
