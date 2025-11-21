# The following variables contains the files used by the different stages of the build process.
set(remote_default_XC8_FILE_TYPE_assemble)
set_source_files_properties(${remote_default_XC8_FILE_TYPE_assemble} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${remote_default_XC8_FILE_TYPE_assemble})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(remote_default_XC8_FILE_TYPE_assemblePreprocess)
set_source_files_properties(${remote_default_XC8_FILE_TYPE_assemblePreprocess} PROPERTIES LANGUAGE ASM)

# For assembly files, add "." to the include path for each file so that .include with a relative path works
foreach(source_file ${remote_default_XC8_FILE_TYPE_assemblePreprocess})
        set_source_files_properties(${source_file} PROPERTIES INCLUDE_DIRECTORIES "$<PATH:NORMAL_PATH,$<PATH:REMOVE_FILENAME,${source_file}>>")
endforeach()

set(remote_default_XC8_FILE_TYPE_compile
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../RFM69.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../RFM69_pic12_platform.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/adc/src/adc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/fvr/src/fvr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/nvm/src/nvm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/spi/src/mssp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/clock.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/config_bits.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/interrupt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/pins.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../mcc_generated_files/system/src/system.c")
set_source_files_properties(${remote_default_XC8_FILE_TYPE_compile} PROPERTIES LANGUAGE C)
set(remote_default_XC8_FILE_TYPE_link)
set(remote_default_image_name "default.elf")
set(remote_default_image_base_name "default")

# The output directory of the final image.
set(remote_default_output_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../out/remote")

# The full path to the final image.
set(remote_default_full_path_to_image ${remote_default_output_dir}/${remote_default_image_name})
