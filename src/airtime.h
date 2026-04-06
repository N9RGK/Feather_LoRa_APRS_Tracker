#pragma once

// Recompute minimum safe transmit intervals from current radio config.
// Defined in main.cpp. Call at boot and after any config change (e.g. APPLY).
void recompute_airtime_limits();
