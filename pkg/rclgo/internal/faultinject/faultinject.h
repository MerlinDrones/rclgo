/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

#ifndef RCLGO_FAULTINJECT_H
#define RCLGO_FAULTINJECT_H

#include <stddef.h>

// rclgo_fi_should_fail counts a call to function name and reports whether it
// is the one to fail. If so, it sets *ret to the code to return and sets the
// rcutils error state, as a failing rcl function does.
int rclgo_fi_should_fail(const char *name, int *ret);

void rclgo_fi_arm(long n, const char *only, int ret);
long rclgo_fi_disarm(char *failed, size_t len);

// rclgo_fi_record starts (on != 0) or stops counting calls per function;
// rclgo_fi_recorded returns the i-th function counted and its calls.
void rclgo_fi_record(int on);
int rclgo_fi_recorded(int i, const char **name, long *count);

// rclgo_fi_logged returns the first log message since rclgo_fi_arm that
// carries an injected error, or "".
void rclgo_fi_logged(char *out, size_t len);

#endif
