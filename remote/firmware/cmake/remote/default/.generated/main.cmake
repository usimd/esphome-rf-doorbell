# cmake files support debug production
include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(remote_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group XC8
if(remote_default_XC8_FILE_TYPE_assemble)
add_library(remote_default_XC8_assemble OBJECT ${remote_default_XC8_FILE_TYPE_assemble})
    remote_default_XC8_assemble_rule(remote_default_XC8_assemble)
    list(APPEND remote_default_library_list "$<TARGET_OBJECTS:remote_default_XC8_assemble>")

endif()

# Handle files with suffix S, for group XC8
if(remote_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(remote_default_XC8_assemblePreprocess OBJECT ${remote_default_XC8_FILE_TYPE_assemblePreprocess})
    remote_default_XC8_assemblePreprocess_rule(remote_default_XC8_assemblePreprocess)
    list(APPEND remote_default_library_list "$<TARGET_OBJECTS:remote_default_XC8_assemblePreprocess>")

endif()

# Handle files with suffix [cC], for group XC8
if(remote_default_XC8_FILE_TYPE_compile)
add_library(remote_default_XC8_compile OBJECT ${remote_default_XC8_FILE_TYPE_compile})
    remote_default_XC8_compile_rule(remote_default_XC8_compile)
    list(APPEND remote_default_library_list "$<TARGET_OBJECTS:remote_default_XC8_compile>")

endif()


# Main target for this project
add_executable(remote_default_image_Fz4LVqVb ${remote_default_library_list})

set_target_properties(remote_default_image_Fz4LVqVb PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${remote_default_output_dir})
set_target_properties(remote_default_image_Fz4LVqVb PROPERTIES OUTPUT_NAME "default")
set_target_properties(remote_default_image_Fz4LVqVb PROPERTIES SUFFIX ".elf")

target_link_libraries(remote_default_image_Fz4LVqVb PRIVATE ${remote_default_XC8_FILE_TYPE_link})


# Add the link options from the rule file.
remote_default_link_rule(remote_default_image_Fz4LVqVb)



