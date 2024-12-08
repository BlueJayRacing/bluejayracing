if(DEFINED CMAKE_FUNCTIONS_INCLUDED)
    return()
endif()
set(CMAKE_FUNCTIONS_INCLUDED TRUE)

# Function to get filenames from a directory
# Usage:
# get_directory_files(
#   DIRECTORY "path/to/dir"          # Required: Directory to scan
#   OUTPUT_VARIABLE my_files         # Required: Variable to store results
#   RECURSIVE                        # Optional: Include subdirectories
#   PATTERN "*.cpp;*.h"             # Optional: File patterns to match (glob)
#   EXCLUDE_PATTERN "*.txt;*.temp"   # Optional: Patterns to exclude
# )
function(get_directory_files)
    # Parse function arguments
    set(options RECURSIVE)
    set(oneValueArgs DIRECTORY OUTPUT_VARIABLE)
    set(multiValueArgs PATTERN EXCLUDE_PATTERN)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Validate required arguments
    if(NOT ARG_DIRECTORY)
        message(FATAL_ERROR "DIRECTORY argument is required")
    endif()
    if(NOT ARG_OUTPUT_VARIABLE)
        message(FATAL_ERROR "OUTPUT_VARIABLE argument is required")
    endif()

    # Set default pattern if none provided
    if(NOT ARG_PATTERN)
        set(ARG_PATTERN "*")
    endif()

    # Build glob expression
    if(ARG_RECURSIVE)
        set(glob_expr "GLOB_RECURSE")
    else()
        set(glob_expr "GLOB")
    endif()

    # Get all files matching pattern
    set(files_list "")
    foreach(pattern ${ARG_PATTERN})
        file(${glob_expr} pattern_files 
            ${ARG_DIRECTORY}/${pattern}
        )
        list(APPEND files_list ${pattern_files})
    endforeach()

    # Remove excluded files
    if(ARG_EXCLUDE_PATTERN)
        foreach(exclude_pattern ${ARG_EXCLUDE_PATTERN})
            file(${glob_expr} exclude_files 
                ${ARG_DIRECTORY}/${exclude_pattern}
            )
            if(exclude_files)
                list(REMOVE_ITEM files_list ${exclude_files})
            endif()
        endforeach()
    endif()

    # Extract just the filenames
    set(final_list "")
    foreach(file ${files_list})
        get_filename_component(filename ${file} NAME)
        list(APPEND final_list ${filename})
    endforeach()

    # Remove duplicates that might occur in recursive mode
    list(REMOVE_DUPLICATES final_list)

    # Return results in the provided variable
    set(${ARG_OUTPUT_VARIABLE} ${final_list} PARENT_SCOPE)
endfunction()

# Function to remove both prefix and postfix from filenames
function(remove_prefix_postfix input_list prefix postfix output_var)
    set(result_list "")
    foreach(item IN LISTS input_list)
        # Remove prefix
        string(LENGTH "${prefix}" prefix_length)
        string(LENGTH "${item}" item_length)
        
        if(prefix_length GREATER 0)
            string(SUBSTRING "${item}" 0 ${prefix_length} start)
            if("${start}" STREQUAL "${prefix}")
                string(SUBSTRING "${item}" ${prefix_length} -1 item)
            endif()
        endif()
        
        # Remove postfix
        if(postfix)
            string(LENGTH "${postfix}" postfix_length)
            string(LENGTH "${item}" item_length)
            if(item_length GREATER postfix_length)
                math(EXPR start_pos "${item_length} - ${postfix_length}")
                string(SUBSTRING "${item}" ${start_pos} -1 end)
                if("${end}" STREQUAL "${postfix}")
                    string(SUBSTRING "${item}" 0 ${start_pos} item)
                endif()
            endif()
        endif()
        
        list(APPEND result_list "${item}")
    endforeach()
    
    set(${output_var} "${result_list}" PARENT_SCOPE)
endfunction()

# Function to add both prefix and postfix to filenames
function(add_prefix_postfix RESULT_VAR SOURCE_LIST)
    # Parse arguments
    set(options "")
    set(oneValueArgs PREFIX POSTFIX)
    set(multiValueArgs "")
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # Set default empty values if not provided
    if(NOT DEFINED ARG_PREFIX)
        set(ARG_PREFIX "")
    endif()
    if(NOT DEFINED ARG_POSTFIX)
        set(ARG_POSTFIX "")
    endif()
    
    set(temp_list "")
    foreach(item ${SOURCE_LIST})
        # Get the filename without directory path
        get_filename_component(filename ${item} NAME)
        # Get just the directory path
        get_filename_component(dir ${item} DIRECTORY)
        # Get the extension
        get_filename_component(ext ${item} EXT)
        # Get the filename without extension
        get_filename_component(name_we ${item} NAME_WE)
        
        # Add prefix and postfix
        if(ext)
            set(new_filename "${ARG_PREFIX}${name_we}${ARG_POSTFIX}${ext}")
        else()
            set(new_filename "${ARG_PREFIX}${filename}${ARG_POSTFIX}")
        endif()
        
        # Reconstruct the full path if there was a directory
        if(dir)
            list(APPEND temp_list "${dir}/${new_filename}")
        else()
            list(APPEND temp_list "${new_filename}")
        endif()
    endforeach()
    
    set(${RESULT_VAR} ${temp_list} PARENT_SCOPE)
endfunction()

function(write_files file_list content_list base_dir)
    list(LENGTH file_list file_count)
    list(LENGTH content_list content_count)
    
    # Check if lists have same length
    if(NOT file_count EQUAL content_count)
        message(FATAL_ERROR "Number of files (${file_count}) doesn't match number of contents (${content_count})")
        return()
    endif()
    
    # Create base directory if it doesn't exist
    file(MAKE_DIRECTORY "${base_dir}")
    
    # Write each file
    math(EXPR last_index "${file_count} - 1")
    foreach(index RANGE ${last_index})
        list(GET file_list ${index} filename)
        list(GET content_list ${index} content)
        
        set(full_path "${base_dir}/${filename}")
        message("Writing file: ${full_path}")
        
        # Create subdirectories if needed
        get_filename_component(dir "${full_path}" DIRECTORY)
        file(MAKE_DIRECTORY "${dir}")
        
        # Write the file
        file(WRITE "${full_path}" "${content}")
    endforeach()
endfunction()

function(clean_directory_preserve dir)
    # Get all files in directory
    file(GLOB files "${dir}/*")
    
    foreach(file ${files})
        if(EXISTS "${file}")
            if(IS_DIRECTORY "${file}")
                clean_directory_preserve("${file}")  # Recursively clean subdirectories
            else()
                file(REMOVE "${file}")
                message("Removed file: ${file}")
            endif()
        endif()
    endforeach()
endfunction()