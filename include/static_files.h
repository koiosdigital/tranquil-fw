#pragma once
// Fallback static_files.h when download fails
namespace static_files {
    struct file {
        const char* path;
        const unsigned char* contents;
        const char* type;
        size_t size;
    };
    
    // Empty files array as fallback
    const file files[] = {};
    const int num_of_files = 0;
}
