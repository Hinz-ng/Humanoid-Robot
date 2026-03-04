#ifndef OE_CONTROL_H
#define OE_CONTROL_H

// =============================================================================
//  OE CONTROL  —  PCA9685 Output Enable pin management
// =============================================================================
//  Hardware:  Both PCA9685 OE pins wired together to GPIO2.
//  Logic:     Active-low. OE HIGH = outputs DISABLED. OE LOW = outputs ENABLED.
//
//  Boot sequence (non-blocking):
//    oe_begin() called in setup() → sets OE HIGH immediately
//    oe_loop()  called in loop()  → releases to LOW after OE_BOOT_HOLD_MS
//
//  Emergency stop:
//    oe_estop()  → OE HIGH immediately (disables both boards)
//    oe_clear()  → OE LOW  (re-enables after user confirmation in UI)
// =============================================================================

#include <Arduino.h>

#define OE_PIN           2       // GPIO2, shared between both PCA9685 OE pins
#define OE_BOOT_HOLD_MS  1000    // Hold outputs disabled for this many ms after boot

// Implemented in main.cpp — extern so WebComm.cpp can call oe_estop/oe_clear
void oe_begin();          // Call once in setup() — sets OE HIGH immediately
void oe_loop();           // Call every loop() — releases OE LOW after boot hold
void oe_estop();          // Immediately disable outputs (OE HIGH)
void oe_clear();          // Re-enable outputs (OE LOW)
bool oe_is_estopped();    // Returns true while e-stopped or boot hold active

#endif // OE_CONTROL_H