// =============================================================================
//  wdt.h  —  Task Watchdog Timer (TWDT) wiring shared by every task.
// =============================================================================
//  If any task (Arduino loop, the LVGL task, or the K-line task) stops making
//  progress — a deadlock, a wedged peripheral, a genuine infinite loop — the
//  device otherwise just hangs forever with nothing to recover it. Subscribing
//  every long-running task and feeding the watchdog from its normal wait
//  points turns that into a reset instead.
//
//  A single flat timeout works here only because every blocking wait in this
//  codebase that can legitimately run long (K-line response timeouts/retries,
//  the up-to-20s-per-target touch calibration) already loops in small steps
//  (a few ms at a time) — each of those loop iterations feeds the watchdog
//  too, so the *overall* wait can be long without ever going quiet for a
//  whole timeout period. Only a real hang (nothing looping, nothing feeding)
//  trips it.
// =============================================================================
#pragma once

#include <stdint.h>

// Initializes the TWDT (idempotent — fine even if the Arduino core already
// set one up at startup) and subscribes the calling task. Call once, early,
// from the task that runs setup()/loop() (main.cpp).
void wdt_init(uint32_t timeout_ms = 8000);

// Subscribes the calling task to the already-initialized watchdog. Call once
// at the very top of any other long-running task's entry function (the LVGL
// task, the K-line task, ...) before its main loop.
void wdt_add_self();

// Feeds the watchdog on behalf of the calling task. Call this from every
// subscribed task's own wait/poll loops, often enough that the gap between
// calls never approaches the configured timeout.
void wdt_feed();
