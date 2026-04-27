#ifndef USEC_TIME_H
#define USEC_TIME_H

#include <stdint.h>

// ─── usec_time ────────────────────────────────────────────────────────────────
// Monotonic microsecond timestamp utilities for the flight loop.
//
// Problem with raw micros():
//   micros() returns uint32_t — overflows back to 0 after ~71 minutes.
//   Casting to uint64_t AFTER the call doesn't prevent the overflow;
//   it just zero-extends an already-wrapped value.
//
// Solution:
//   usecTimestamp() accumulates into a true uint64_t by tracking the
//   previous uint32_t value and adding the delta each call.
//   This gives ~585,000 years of range.
//
// Usage:
//   Call usecTimestamp() at least once every 71 minutes (the control loop
//   runs at 500Hz so this is always satisfied in practice).

// ─── usecTimestamp ────────────────────────────────────────────────────────────
// Returns a monotonically increasing 64-bit microsecond timestamp.
// First call initializes the base. Never rolls over in practice.
uint64_t usecTimestamp();

// ─── usecTimestampDelta ───────────────────────────────────────────────────────
// Returns microseconds elapsed since `previousUs`.
// Handles the uint32_t overflow boundary transparently.
// Usage:
//   uint64_t last = usecTimestamp();
//   ... do work ...
//   float dt = usecTimestampDelta(last) / 1000000.0f;
uint64_t usecTimestampDelta(uint64_t previousUs);

// ─── usecSleep ────────────────────────────────────────────────────────────────
// Busy-wait until `durationUs` microseconds have elapsed.
// More precise than delay() for sub-millisecond timing.
void usecSleep(uint32_t durationUs);

#endif // USEC_TIME_H