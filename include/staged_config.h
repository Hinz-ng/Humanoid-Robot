// =============================================================================
// FILE:    staged_config.h
// MODULE:  comm / control boundary
// LAYER:   0 — vocabulary/utility (no robot dependencies; <atomic> only)
//
// PURPOSE:
//   Single-producer / single-consumer staged double-buffer for the runtime-
//   tunable config structs that cross the AsyncTCP (WiFi) task → 400 Hz
//   main-task boundary. Replaces the racy full-struct read-modify-write
//   (getConfig() → modify → setConfig()) the WebSocket parsers used to do
//   directly on live controller state (audit finding A-01 / fix B1).
//
//   Producer (WiFi task, WebComm parsers):
//     T c = staged.beginEdit(module->getConfig());   // RMW base
//     ...modify fields...
//     staged.publish(c);                             // stage, do NOT apply
//
//   Consumer (main task, once per tick at the TOP of the 400 Hz gate,
//   before any controller reads its config):
//     T c; if (staged.consume(c)) module->setConfig(c);
//
// CONCURRENCY CONTRACT:
//   - Exactly ONE producer task and ONE consumer task per instance.
//   - The buffer is protected by a sequence counter (seqlock): the producer
//     makes it odd while writing, even when stable. The consumer copies the
//     buffer only when the counter is even and unchanged across the copy;
//     otherwise it keeps the previous config for one more tick and retries.
//     No locks, no blocking, no priority inversion in the 400 Hz hot path —
//     the only cost when idle is one relaxed atomic load per struct per tick.
//   - beginEdit() returns the last staged-but-not-yet-applied value when one
//     is pending, so two tune commands landing inside the same tick compose
//     instead of the second clobbering the first.
//   - Worst-case races are benign by construction: a publish overlapping a
//     consume either delays the apply by one tick (2.5 ms) or applies the
//     same struct twice (idempotent). A staged update can never be applied
//     torn, and can never be lost.
//
//   Existing targeted atomic setters (SteeringController, WeightShift hot
//   path) and FootContactConfig's in-place per-field stores are deliberately
//   NOT routed through this — they are already cross-task safe by design.
//
// LAST CHANGED: 2026-07-07 | audit/v1-closure B1 | command staging
// =============================================================================

#ifndef STAGED_CONFIG_H
#define STAGED_CONFIG_H

#include <atomic>
#include <stdint.h>

template <typename T>
class StagedConfig {
public:
    // Producer (WiFi task) — base value for a read-modify-write edit.
    // Returns the pending staged value if one exists (so successive edits in
    // the same tick compose), otherwise the caller-supplied live config.
    // Safe: _buf is only ever written by this same task.
    T beginEdit(const T& live) {
        if (_pending.load(std::memory_order_acquire)) return _buf;
        return live;
    }

    // Producer (WiFi task) — stage a new config for the main task to apply.
    // Never applies anything itself.
    void publish(const T& v) {
        _seq.fetch_add(1, std::memory_order_acq_rel);   // odd → write in progress
        _buf = v;
        _seq.fetch_add(1, std::memory_order_acq_rel);   // even → stable
        _pending.store(true, std::memory_order_release);
    }

    // Consumer (main task) — cheap per-tick fast-path check.
    bool hasPending() const {
        return _pending.load(std::memory_order_acquire);
    }

    // Consumer (main task) — copy the staged value out if it is stable.
    // Returns false (and leaves `out` untouched) when nothing is pending or
    // the producer is mid-write; in the latter case pending is re-armed so
    // the next tick retries. A publish that completes between the claim and
    // the copy re-arms pending itself, so the newer value is applied next
    // tick (the current copy is then a same-or-older full struct — never torn,
    // because the seq check below rejects any copy that overlapped a write).
    bool consume(T& out) {
        if (!_pending.load(std::memory_order_acquire)) return false;
        _pending.store(false, std::memory_order_release);          // claim
        const uint32_t s1 = _seq.load(std::memory_order_acquire);
        if (s1 & 1u) {                                             // writer active
            _pending.store(true, std::memory_order_release);       // re-arm
            return false;
        }
        T copy = _buf;
        const uint32_t s2 = _seq.load(std::memory_order_acquire);
        if (s1 != s2) {                                            // copy overlapped a write
            _pending.store(true, std::memory_order_release);       // re-arm
            return false;
        }
        out = copy;
        return true;
    }

private:
    T                     _buf{};
    std::atomic<uint32_t> _seq{0};
    std::atomic<bool>     _pending{false};
};

#endif // STAGED_CONFIG_H
