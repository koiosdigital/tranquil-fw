#pragma once

#ifndef KD_COMMON_CONSOLE_DISABLE

namespace sand_table {

class MotionController;

// Initialize sand_table console commands
// Call after MotionController is initialized
void console_init(MotionController* controller);

} // namespace sand_table

#endif // KD_COMMON_CONSOLE_DISABLE
