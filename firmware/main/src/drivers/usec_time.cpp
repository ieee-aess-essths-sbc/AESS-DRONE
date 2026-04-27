#include "usec_time.h"
#include <Arduino.h>

// ─── Module state ─────────────────────────────────────────────────────────────
// prev32  : last raw micros() value — used to detect the 32-bit rollover
// base64  : cumulative high-bits — adds 2^32 each time micros() wraps
static uint32_t prev32 = 0;
static uint64_t base64 = 0;
static bool     initialized = false;

// ─── usecTimestamp ────────────────────────────────────────────────────────────
// Each call checks if micros() has wrapped (new < previous).
// If so, it adds 2^32 to the 64-bit accumulator before computing the result.
// This works correctly as long as the function is called at least once
// per 71-minute rollover window — guaranteed by the 500Hz control loop.
uint64_t usecTimestamp() {
    uint32_t now32 = micros();

    if (!initialized) {
        // First call — seed the base with the current raw value
        prev32      = now32;
        base64      = (uint64_t)now32;
        initialized = true;
        return base64;
    }

    if (now32 < prev32) {
        // micros() has wrapped — add one full 32-bit period to the accumulator
        base64 += (uint64_t)0x100000000ULL; // = 2^32 microseconds ≈ 71.6 minutes
    }

    prev32 = now32;

    // Result: high bits from base64 (accumulated wraps) + current low 32 bits
    return (base64 & 0xFFFFFFFF00000000ULL) | (uint64_t)now32;
}

// ─── usecTimestampDelta ───────────────────────────────────────────────────────
// Simple subtraction on 64-bit values — no special overflow handling needed
// because usecTimestamp() already manages the wrap.
uint64_t usecTimestampDelta(uint64_t previousUs) {
    return usecTimestamp() - previousUs;
}

// ─── usecSleep ────────────────────────────────────────────────────────────────
// Busy-wait for precise short delays (e.g. between calibration samples).
// Uses 32-bit arithmetic — fine here since durationUs is always <1s in practice.
void usecSleep(uint32_t durationUs) {
    uint32_t start = micros();
    // Subtraction handles the 32-bit wrap correctly here:
    // if micros() wraps mid-sleep, (micros()-start) still gives the right delta
    while ((micros() - start) < durationUs) {
        // busy wait — intentionally empty
    }
}