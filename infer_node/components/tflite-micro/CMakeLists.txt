## TODO: GLOB is not a good way to collect files. Use explicit file list instead

cmake_minimum_required(VERSION 3.5)

set(tflite_dir "${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/lite")
set(signal_dir "${CMAKE_CURRENT_SOURCE_DIR}/signal")
set(compiler_mlir_dir "${CMAKE_CURRENT_SOURCE_DIR}/tensorflow/compiler/mlir/")
set(tfmicro_dir "${tflite_dir}/micro")
set(tfmicro_frontend_dir "${tflite_dir}/experimental/microfrontend/lib")
set(tfmicro_kernels_dir "${tfmicro_dir}/kernels")
set(tfmicro_compiler_dir "${tfmicro_dir}/compiler")

set(srcs_micro
          "${tfmicro_dir}/debug_log.cc"
          "${tfmicro_dir}/flatbuffer_utils.cc"
          "${tfmicro_dir}/memory_helpers.cc"
          "${tfmicro_dir}/micro_allocation_info.cc"
          "${tfmicro_dir}/micro_allocator.cc"
          "${tfmicro_dir}/micro_context.cc"
          "${tfmicro_dir}/micro_interpreter_context.cc"
          "${tfmicro_dir}/micro_interpreter_graph.cc"
          "${tfmicro_dir}/micro_interpreter.cc"
          "${tfmicro_dir}/micro_log.cc"
          "${tfmicro_dir}/micro_op_resolver.cc"
          "${tfmicro_dir}/micro_profiler.cc"
          "${tfmicro_dir}/micro_resource_variable.cc"
          "${tfmicro_dir}/micro_time.cc"
          "${tfmicro_dir}/micro_utils.cc"
          "${tfmicro_dir}/recording_micro_allocator.cc"
          "${tfmicro_dir}/system_setup.cc")

list(REMOVE_ITEM srcs_micro
          "${tfmicro_dir}/micro_time.cc")
list(APPEND srcs_micro
          "${tfmicro_dir}/esp/micro_time.cc")

file(GLOB src_micro_frontend
          "${tfmicro_frontend_dir}/*.c"
          "${tfmicro_frontend_dir}/*.cc")

file(GLOB srcs_tflite_bridge
          "${tfmicro_dir}/tflite_bridge/*.c"
          "${tfmicro_dir}/tflite_bridge/*.cc")

file(GLOB srcs_kernels
          "${tfmicro_kernels_dir}/*.c"
          "${tfmicro_kernels_dir}/*.cc")

file(GLOB signal_micro_kernels
          "${signal_dir}/micro/kernels/*.c"
          "${signal_dir}/micro/kernels/*.cc")

file(GLOB signal_src
          "${signal_dir}/src/*.c"
          "${signal_dir}/src/*.cc")

set(signal_srcs
          "${signal_micro_kernels}"
          "${signal_src}"
          "${signal_dir}/src/kiss_fft_wrappers/kiss_fft_float.cc"
          "${signal_dir}/src/kiss_fft_wrappers/kiss_fft_int16.cc"
          "${signal_dir}/src/kiss_fft_wrappers/kiss_fft_int32.cc")

# did_node: choose which kernels use ESP-NN (top-level CMakeLists.txt owns the
# ESP_NN_KERNEL_SET cache variable; it is undefined while IDF scans requirements).
#   (ESP32 infer_node: the top level resolves its default "auto" to all, for both roles.)
#   none: all TFLM reference kernels. FD_did_cam_v4 got a constant
#   softmax output with ESP-NN on the ESP32, so nothing relies on it until the
#   S3 build has been compared against PC TFLite (docs/inference_integration_plan.md).
#   all / dw / conv / fc / arith / pool / softmax: replace those reference kernels.
set(esp_nn_names "")
if(DEFINED ESP_NN_KERNEL_SET)
  if(ESP_NN_KERNEL_SET STREQUAL "all")
    set(esp_nn_names add conv depthwise_conv fully_connected mul pooling softmax)
  elseif(ESP_NN_KERNEL_SET STREQUAL "dw")
    set(esp_nn_names depthwise_conv)
  elseif(ESP_NN_KERNEL_SET STREQUAL "conv")
    set(esp_nn_names conv)
  elseif(ESP_NN_KERNEL_SET STREQUAL "fc")
    set(esp_nn_names fully_connected)
  elseif(ESP_NN_KERNEL_SET STREQUAL "arith")
    set(esp_nn_names add mul)
  elseif(ESP_NN_KERNEL_SET STREQUAL "pool")
    set(esp_nn_names pooling)
  elseif(ESP_NN_KERNEL_SET STREQUAL "softmax")
    set(esp_nn_names softmax)
  elseif(NOT ESP_NN_KERNEL_SET STREQUAL "none")
    message(FATAL_ERROR "Unknown ESP_NN_KERNEL_SET=${ESP_NN_KERNEL_SET} (expected none/all/dw/conv/fc/arith/pool/softmax)")
  endif()
endif()

set(esp_nn_kernels "")
foreach(name IN LISTS esp_nn_names)
  list(REMOVE_ITEM srcs_kernels "${tfmicro_kernels_dir}/${name}.cc")
  list(APPEND esp_nn_kernels "${tfmicro_kernels_dir}/esp_nn/${name}.cc")
endforeach()

set(lib_srcs
          "${srcs_micro}"
          "${srcs_kernels}"
          "${srcs_tflite_bridge}"
          "${esp_nn_kernels}"
          "${src_micro_frontend}"
          "${signal_srcs}"
          "${tflite_dir}/kernels/kernel_util.cc"
          "${tflite_dir}/micro/memory_planner/greedy_memory_planner.cc"
          "${tflite_dir}/micro/memory_planner/linear_memory_planner.cc"
          "${tflite_dir}/micro/arena_allocator/non_persistent_arena_buffer_allocator.cc"
          "${tflite_dir}/micro/arena_allocator/persistent_arena_buffer_allocator.cc"
          "${tflite_dir}/micro/arena_allocator/recording_single_arena_buffer_allocator.cc"
          "${tflite_dir}/micro/arena_allocator/single_arena_buffer_allocator.cc"
          "${tflite_dir}/core/c/common.cc"
          "${tflite_dir}/core/api/flatbuffer_conversions.cc"
          "${tflite_dir}/core/api/tensor_utils.cc"
          "${tflite_dir}/kernels/internal/common.cc"
          "${tflite_dir}/kernels/internal/quantization_util.cc"
          "${tflite_dir}/kernels/internal/portable_tensor_utils.cc"
          "${tflite_dir}/kernels/internal/tensor_utils.cc"
          "${tflite_dir}/kernels/internal/tensor_ctypes.cc"
          "${tflite_dir}/kernels/internal/reference/portable_tensor_utils.cc"
          "${tflite_dir}/kernels/internal/reference/comparisons.cc"
          "${compiler_mlir_dir}/lite/core/api/error_reporter.cc"
          "${compiler_mlir_dir}/lite/schema/schema_utils.cc")

set(priv_req)

# include component requirements which were introduced after IDF version 4.1
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND priv_req esp_timer driver)
endif()

idf_component_register(
            SRCS "${lib_srcs}"
            INCLUDE_DIRS "." "third_party/gemmlowp"
                         "third_party/flatbuffers/include"
                         "third_party/ruy"
                         "third_party/kissfft"
            REQUIRES ${pub_req}
            PRIV_REQUIRES ${priv_req})

# Linked only when selected, so the reference build never touches its code.
if(esp_nn_names)
  target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp-nn)
endif()

# Reduce the level of paranoia to be able to compile TF sources
target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wno-error=attributes
  -Wno-error=shadow
  -Wno-maybe-uninitialized
  -Wno-missing-field-initializers
  -Wno-error=sign-compare
  -Wno-error=double-promotion
  -Wno-type-limits)

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
  target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=stringop-overread)
endif()

# enable ESP-NN optimizations by Espressif
target_compile_options(${COMPONENT_LIB} PRIVATE -DESP_NN)

# did_node: the optimization level depends on the project setting.
#   COMPILER_OPTIMIZATION_PERF (-O2 project): upstream -O3, with depthwise_conv.cc
#     at -O2 (below). Plain -O2 is not an option: GCC 12.2 hits an internal
#     compiler error on reference/portable_tensor_utils.cc at -O2 (-O1/-O3/-Os build).
#   otherwise (-Og for debugging): follow the project, nothing added here.
if(CONFIG_COMPILER_OPTIMIZATION_PERF)
  set(tflm_opt_flags -O3)
else()
  set(tflm_opt_flags "")
endif()
set(common_flags -DTF_LITE_STATIC_MEMORY -DTF_LITE_DISABLE_X86_NEON ${tflm_opt_flags}
                 -Wstrict-aliasing -Wno-unused-parameter -Wall -Wextra -Wvla
                 -Wsign-compare -Wdouble-promotion -Wswitch -Wunused-function
                 -Wmissing-field-initializers -ffunction-sections -fdata-sections
                 -Wshadow -Wunused-variable -fno-unwind-tables -fmessage-length=0)

target_compile_options(${COMPONENT_LIB} PRIVATE ${common_flags} -fmessage-length=0 -Wno-nonnull)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:
                                      ${common_flags} -std=c++11 -fno-rtti -fno-exceptions
                                      -fno-threadsafe-statics -Werror -Wno-return-type
                                      -Wno-strict-aliasing -std=gnu++14 >)

# avoid issue of compiler internal error caused in `depthwise_conv_flaot.h
# (the reference kernel includes the same header and crashes GCC 12.2 at -O3 too)
if(CONFIG_COMPILER_OPTIMIZATION_PERF)
  set_source_files_properties(${tfmicro_kernels_dir}/esp_nn/depthwise_conv.cc
                              ${tfmicro_kernels_dir}/depthwise_conv.cc
                              PROPERTIES COMPILE_FLAGS -O2)
endif()

target_compile_options(${COMPONENT_LIB} INTERFACE -DTF_LITE_STATIC_MEMORY)
target_link_libraries(${COMPONENT_LIB} PRIVATE -lm)
