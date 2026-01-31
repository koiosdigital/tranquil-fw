# tranquil-app component CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Set component name
set(COMPONENT_NAME "tranquil-app")

# Define the GitHub repository details
set(GITHUB_REPO "koiosdigital/tranquil-app")
set(STATIC_FILES_HEADER "static_files.h")
set(DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/downloads")
set(INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(STATIC_FILES_PATH "${INCLUDE_DIR}/${STATIC_FILES_HEADER}")

# Create directories early - this is safe in early expansion
file(MAKE_DIRECTORY ${DOWNLOAD_DIR})
file(MAKE_DIRECTORY ${INCLUDE_DIR})

# Function to get the latest release tag
function(get_latest_release_tag REPO_NAME OUTPUT_VAR)
    if(DEFINED ENV{GITHUB_TOKEN})
        execute_process(
            COMMAND curl -s -H "Authorization: token $ENV{GITHUB_TOKEN}" "https://api.github.com/repos/${REPO_NAME}/releases/latest"
            COMMAND grep "tag_name"
            COMMAND head -n 1
            COMMAND cut -d ":" -f 2
            COMMAND sed "s/[\", ]//g"
            OUTPUT_VARIABLE LATEST_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE CURL_RESULT
        )
    else()
        execute_process(
            COMMAND curl -s "https://api.github.com/repos/${REPO_NAME}/releases/latest"
            COMMAND grep "tag_name"
            COMMAND head -n 1
            COMMAND cut -d ":" -f 2
            COMMAND sed "s/[\", ]//g"
            OUTPUT_VARIABLE LATEST_TAG
            OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE CURL_RESULT
        )
    endif()
    
    if(CURL_RESULT EQUAL 0 AND NOT "${LATEST_TAG}" STREQUAL "")
        set(${OUTPUT_VAR} ${LATEST_TAG} PARENT_SCOPE)
        message(STATUS "Latest release tag for ${REPO_NAME}: ${LATEST_TAG}")
    else()
        set(${OUTPUT_VAR} "main" PARENT_SCOPE)
        message(WARNING "Failed to get latest release tag for ${REPO_NAME}, using 'main' branch")
    endif()
endfunction()

# Fallback content for static_files.h
set(FALLBACK_CONTENT "#pragma once
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
")

# Always create a basic fallback file first - this is safe in early expansion
if(NOT EXISTS ${STATIC_FILES_PATH})
    file(WRITE ${STATIC_FILES_PATH} "${FALLBACK_CONTENT}")
    message(STATUS "Created initial fallback static_files.h")
endif()

# Register the component early with basic configuration
# ESP-IDF needs this to happen before the expansion boundary
idf_component_register(
    INCLUDE_DIRS ${INCLUDE_DIR}
    REQUIRES esp_http_server
)

# ESP-IDF early expansion boundary - everything after this only runs during actual build
if(NOT CMAKE_BUILD_EARLY_EXPANSION)

    message(STATUS "Processing tranquil-app component download logic...")

    # Get the latest release tag
    get_latest_release_tag(${GITHUB_REPO} LATEST_TAG)

    # Download URL for the static_files.h
    if("${LATEST_TAG}" STREQUAL "main")
        # Use raw GitHub URL for main branch
        set(DOWNLOAD_URL "https://raw.githubusercontent.com/${GITHUB_REPO}/main/${STATIC_FILES_HEADER}")
    else()
        # Use release download URL
        set(DOWNLOAD_URL "https://github.com/${GITHUB_REPO}/releases/download/${LATEST_TAG}/${STATIC_FILES_HEADER}")
    endif()

    # Try to download the file during actual build configuration
    execute_process(
        COMMAND curl -L -f -o ${STATIC_FILES_PATH} ${DOWNLOAD_URL}
        RESULT_VARIABLE DOWNLOAD_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )

    # Check if download was successful and file has content
    if(DOWNLOAD_RESULT EQUAL 0 AND EXISTS ${STATIC_FILES_PATH})
        file(SIZE ${STATIC_FILES_PATH} FILE_SIZE)
        if(FILE_SIZE GREATER 100)  # Reasonable minimum size check
            message(STATUS "Successfully downloaded ${STATIC_FILES_HEADER} from ${GITHUB_REPO} (${LATEST_TAG})")
            file(READ ${STATIC_FILES_PATH} FILE_CONTENT LIMIT 200)
            if(FILE_CONTENT MATCHES "namespace static_files")
                message(STATUS "Downloaded file appears to be valid static_files.h")
            else()
                message(WARNING "Downloaded file may not be valid static_files.h, keeping fallback")
                file(WRITE ${STATIC_FILES_PATH} "${FALLBACK_CONTENT}")
            endif()
        else()
            message(WARNING "Downloaded file is too small, using fallback")
            file(WRITE ${STATIC_FILES_PATH} "${FALLBACK_CONTENT}")
        endif()
    else()
        message(WARNING "Failed to download ${STATIC_FILES_HEADER} from ${DOWNLOAD_URL}, keeping fallback")
    endif()

    file(WRITE ${STATIC_FILES_PATH} "${FALLBACK_CONTENT}")

    # Print detailed status information
    message(STATUS "tranquil-app component configured:")
    message(STATUS "  Repository: ${GITHUB_REPO}")
    message(STATUS "  Target file: ${STATIC_FILES_HEADER}")
    message(STATUS "  Download URL: ${DOWNLOAD_URL}")
    message(STATUS "  Local path: ${STATIC_FILES_PATH}")
    if(EXISTS ${STATIC_FILES_PATH})
        file(SIZE ${STATIC_FILES_PATH} FINAL_FILE_SIZE)
        message(STATUS "  File exists: YES (${FINAL_FILE_SIZE} bytes)")
    else()
        message(STATUS "  File exists: NO")
    endif()

endif() # CMAKE_BUILD_EARLY_EXPANSION