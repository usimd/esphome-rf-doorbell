# The following functions contains all the flags passed to the different build stages.

set(PACK_REPO_PATH "/home/q425942/.mchp_packs" CACHE PATH "Path to the root of a pack repository.")

function(remote_pic16_debug_XC8_assemble_rule target)
    set(options
        "-c"
        "${MP_EXTRA_AS_PRE}"
        "-mcpu=16F18854"
        "${DEBUGGER_NAME}"
        "-mdfp=${PACK_REPO_PATH}/Microchip/PIC16F1xxxx_DFP/1.28.431/xc8"
        "-fno-short-double"
        "-fno-short-float"
        "-O1"
        "-fasmfile"
        "-Og"
        "-maddrqual=require"
        "-mwarn=-3"
        "-Wa,-a"
        "-msummary=-psect,-class,+mem,-hex,-file"
        "-ginhx32"
        "-Wl,--data-init"
        "-mno-keep-startup"
        "-mno-osccal"
        "-mno-resetbits"
        "-mno-save-resetbits"
        "-mno-download"
        "-mno-stackcall"
        "-mno-default-config-bits"
        "-std=c99"
        "-gdwarf-3"
        "-mstack=compiled:auto:auto")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__16F18854__"
        PRIVATE "__DEBUG=1"
        PRIVATE "XPRJ_pic16_debug=pic16_debug")
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
endfunction()
function(remote_pic16_debug_XC8_assemblePreprocess_rule target)
    set(options
        "-c"
        "${MP_EXTRA_AS_PRE}"
        "-mcpu=16F18854"
        "-x"
        "assembler-with-cpp"
        "-mdfp=${PACK_REPO_PATH}/Microchip/PIC16F1xxxx_DFP/1.28.431/xc8"
        "-fno-short-double"
        "-fno-short-float"
        "-O1"
        "-fasmfile"
        "-Og"
        "-maddrqual=require"
        "-mwarn=-3"
        "-Wa,-a"
        "-msummary=-psect,-class,+mem,-hex,-file"
        "-ginhx32"
        "-Wl,--data-init"
        "-mno-keep-startup"
        "-mno-osccal"
        "-mno-resetbits"
        "-mno-save-resetbits"
        "-mno-download"
        "-mno-stackcall"
        "-mno-default-config-bits"
        "-std=c99"
        "-gdwarf-3"
        "-mstack=compiled:auto:auto")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__16F18854__"
        PRIVATE "__DEBUG=1"
        PRIVATE "XPRJ_pic16_debug=pic16_debug")
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
endfunction()
function(remote_pic16_debug_XC8_compile_rule target)
    set(options
        "-c"
        "${MP_EXTRA_CC_PRE}"
        "-mcpu=16F18854"
        "${DEBUGGER_NAME}"
        "-mdfp=${PACK_REPO_PATH}/Microchip/PIC16F1xxxx_DFP/1.28.431/xc8"
        "-fno-short-double"
        "-fno-short-float"
        "-O1"
        "-fasmfile"
        "-Og"
        "-maddrqual=require"
        "-mwarn=-3"
        "-Wa,-a"
        "-msummary=-psect,-class,+mem,-hex,-file"
        "-ginhx32"
        "-Wl,--data-init"
        "-mno-keep-startup"
        "-mno-osccal"
        "-mno-resetbits"
        "-mno-save-resetbits"
        "-mno-download"
        "-mno-stackcall"
        "-mno-default-config-bits"
        "-std=c99"
        "-gdwarf-3"
        "-mstack=compiled:auto:auto")
    list(REMOVE_ITEM options "")
    target_compile_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__16F18854__"
        PRIVATE "__DEBUG=1"
        PRIVATE "XPRJ_pic16_debug=pic16_debug")
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
endfunction()
function(remote_pic16_debug_link_rule target)
    set(options
        "-Wl,-Map=mem.map"
        "${MP_EXTRA_LD_PRE}"
        "-mcpu=16F18854"
        "${DEBUGGER_NAME}"
        "-Wl,--defsym=__MPLAB_BUILD=1"
        "-mdfp=${PACK_REPO_PATH}/Microchip/PIC16F1xxxx_DFP/1.28.431/xc8"
        "-fno-short-double"
        "-fno-short-float"
        "-O1"
        "-fasmfile"
        "-Og"
        "-maddrqual=require"
        "-mwarn=-3"
        "-Wa,-a"
        "-msummary=-psect,-class,+mem,-hex,-file"
        "-ginhx32"
        "-Wl,--data-init"
        "-mno-keep-startup"
        "-mno-osccal"
        "-mno-resetbits"
        "-mno-save-resetbits"
        "-mno-download"
        "-mno-stackcall"
        "-mno-default-config-bits"
        "-std=c99"
        "-gdwarf-3"
        "-mstack=compiled:auto:auto"
        "-Wl,--memorysummary,memoryfile.xml")
    list(REMOVE_ITEM options "")
    target_link_options(${target} PRIVATE "${options}")
    target_compile_definitions(${target}
        PRIVATE "__DEBUG=1"
        PRIVATE "XPRJ_pic16_debug=pic16_debug")
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
endfunction()
