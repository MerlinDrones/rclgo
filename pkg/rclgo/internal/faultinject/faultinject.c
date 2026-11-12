/*
This file is part of rclgo

Copyright © 2026 Merlin Drones

Licensed under the Apache License, Version 2.0 (the "License");
    http://www.apache.org/licenses/LICENSE-2.0
*/

#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <rcutils/error_handling.h>
#include <rcutils/logging.h>

#include "faultinject.h"

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static long countdown; // calls left until the failing one; 0 when disarmed
static long calls;     // calls counted since rclgo_fi_arm
static char only[128]; // if not empty, only calls to this function count
static int ret_code;
static char failed[128]; // the function that was made to fail
static char logged[512]; // the first log message reporting the injected fault

// Calls per function while recording.
#define MAX_FUNCTIONS 256
static int recording;
static const char *rec_names[MAX_FUNCTIONS];
static long rec_counts[MAX_FUNCTIONS];
static int rec_len;

static void record(const char *name)
{
  for (int i = 0; i < rec_len; i++) {
    if (rec_names[i] == name || strcmp(rec_names[i], name) == 0) {
      rec_counts[i]++;
      return;
    }
  }
  if (rec_len < MAX_FUNCTIONS) {
    rec_names[rec_len] = name;  // a string literal in the wrapper
    rec_counts[rec_len++] = 1;
  }
}

int rclgo_fi_should_fail(const char *name, int *ret)
{
  int fail = 0;
  pthread_mutex_lock(&mu);
  if (recording) {
    record(name);
  }
  if (countdown > 0 && (only[0] == '\0' || strcmp(only, name) == 0)) {
    calls++;
    if (--countdown == 0) {
      fail = 1;
      *ret = ret_code;
      snprintf(failed, sizeof(failed), "%s", name);
    }
  }
  pthread_mutex_unlock(&mu);
  if (fail) {
    char msg[192];
    snprintf(msg, sizeof(msg), "injected fault in %s", name);
    rcutils_reset_error();
    rcutils_set_error_state(msg, __FILE__, __LINE__);
  }
  return fail;
}

void rclgo_fi_arm(long n, const char *fn, int ret)
{
  pthread_mutex_lock(&mu);
  countdown = n;
  calls = 0;
  ret_code = ret;
  snprintf(only, sizeof(only), "%s", fn);
  failed[0] = '\0';
  logged[0] = '\0';
  pthread_mutex_unlock(&mu);
}

long rclgo_fi_disarm(char *failed_out, size_t len)
{
  pthread_mutex_lock(&mu);
  long n = calls;
  countdown = 0;
  snprintf(failed_out, len, "%s", failed);
  pthread_mutex_unlock(&mu);
  return n;
}

void rclgo_fi_record(int on)
{
  pthread_mutex_lock(&mu);
  recording = on;
  if (on) {
    rec_len = 0;
  }
  pthread_mutex_unlock(&mu);
}

int rclgo_fi_recorded(int i, const char **name, long *count)
{
  pthread_mutex_lock(&mu);
  int ok = i < rec_len;
  if (ok) {
    *name = rec_names[i];
    *count = rec_counts[i];
  }
  pthread_mutex_unlock(&mu);
  return ok;
}

// rclgo logs with rcutils_log; a message that carries the injected error
// counts as reporting it.
void __real_rcutils_log(const rcutils_log_location_t *location, int severity, const char *name, const char *format, ...);
void __wrap_rcutils_log(const rcutils_log_location_t *location, int severity, const char *name, const char *format, ...)
{
  char msg[2048];
  va_list args;
  va_start(args, format);
  vsnprintf(msg, sizeof(msg), format, args);
  va_end(args);
  if (strstr(msg, "injected fault in ") != NULL) {
    pthread_mutex_lock(&mu);
    if (logged[0] == '\0') {
      snprintf(logged, sizeof(logged), "%.511s", msg);
    }
    pthread_mutex_unlock(&mu);
  }
  __real_rcutils_log(location, severity, name, "%s", msg);
}

void rclgo_fi_logged(char *out, size_t len)
{
  pthread_mutex_lock(&mu);
  snprintf(out, len, "%s", logged);
  pthread_mutex_unlock(&mu);
}
