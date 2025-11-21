# cmake files support debug production
include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(remote_pic16_debug_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group XC8
if(remote_pic16_debug_XC8_FILE_TYPE_assemble)
add_library(remote_pic16_debug_XC8_assemble OBJECT ${remote_pic16_debug_XC8_FILE_TYPE_assemble})
    remote_pic16_debug_XC8_assemble_rule(remote_pic16_debug_XC8_assemble)
    list(APPEND remote_pic16_debug_library_list "$<TARGET_OBJECTS:remote_pic16_debug_XC8_assemble>")
endif()

# Handle files with suffix S, for group XC8
if(remote_pic16_debug_XC8_FILE_TYPE_assemblePreprocess)
add_library(remote_pic16_debug_XC8_assemblePreprocess OBJECT ${remote_pic16_debug_XC8_FILE_TYPE_assemblePreprocess})
    remote_pic16_debug_XC8_assemblePreprocess_rule(remote_pic16_debug_XC8_assemblePreprocess)
    list(APPEND remote_pic16_debug_library_list "$<TARGET_OBJECTS:remote_pic16_debug_XC8_assemblePreprocess>")
endif()

# Handle files with suffix [cC], for group XC8
if(remote_pic16_debug_XC8_FILE_TYPE_compile)
add_library(remote_pic16_debug_XC8_compile OBJECT ${remote_pic16_debug_XC8_FILE_TYPE_compile})
    remote_pic16_debug_XC8_compile_rule(remote_pic16_debug_XC8_compile)
    list(APPEND remote_pic16_debug_library_list "$<TARGET_OBJECTS:remote_pic16_debug_XC8_compile>")
endif()

add_executable(remote_pic16_debug_image_sKpz5Bo_ ${remote_pic16_debug_library_list})

set_target_properties(remote_pic16_debug_image_sKpz5Bo_ PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${remote_pic16_debug_output_dir})
set_target_properties(remote_pic16_debug_image_sKpz5Bo_ PROPERTIES OUTPUT_NAME "pic16_debug")
set_target_properties(remote_pic16_debug_image_sKpz5Bo_ PROPERTIES SUFFIX ".elf")

target_link_libraries(remote_pic16_debug_image_sKpz5Bo_ PRIVATE ${remote_pic16_debug_XC8_FILE_TYPE_link})


# Add the link options from the rule file.
remote_pic16_debug_link_rule(remote_pic16_debug_image_sKpz5Bo_)



