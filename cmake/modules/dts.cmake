# SPDX-License-Identifier: Apache-2.0

include_guard(GLOBAL)

include(extensions)
include(python)
include(boards)
find_package(HostTools)
find_package(Dtc 1.4.6)

# Zephyr code is usually configured using devicetree, but this is
# still technically optional (see e.g. CONFIG_HAS_DTS).
#
# This module makes information from the devicetree available to
# various build stages, as well as to other arbitrary Python scripts:
#
#   - To Zephyr and application source code files, as a C macro API
#     defined in <zephyr/devicetree.h>
#
#   - To other arbitrary Python scripts (like twister) using a
#     serialized edtlib.EDT object in Python's pickle format
#     (https://docs.python.org/3/library/pickle.html)
#
#   - To users as a final devicetree source (DTS) file which can
#     be used for debugging
#
#   - To CMake files, after this module has finished running, using
#     devicetree extensions defined in cmake/modules/extensions.cmake
#
#   - To Kconfig files, both using some Kconfig symbols we generate
#     here as well as the extension functions defined in
#     scripts/kconfig/kconfigfunctions.py
#
# See the specific API documentation for each of these cases for more
# information on what is currently available to you.
#
# We rely on the C preprocessor, the devicetree python package, and
# files in scripts/dts to make all this work. We also optionally will
# run the dtc tool if it is found, in order to catch any additional
# warnings or errors it generates.

# We will place some generated include files in here.
set(BINARY_DIR_INCLUDE           ${PROJECT_BINARY_DIR}/include)
set(BINARY_DIR_INCLUDE_GENERATED ${BINARY_DIR_INCLUDE}/generated)
# Unconditionally create it, even if we don't have DTS support. This
# is a historical artifact, and users expect this directory to exist
# to put their own generated content inside.
file(MAKE_DIRECTORY ${BINARY_DIR_INCLUDE_GENERATED})

# The directory containing devicetree related scripts.
set(DT_SCRIPTS                  ${ZEPHYR_BASE}/scripts/dts)

# This generates DT information needed by the C macro APIs,
# along with a few other things.
set(GEN_DEFINES_SCRIPT          ${DT_SCRIPTS}/gen_defines.py)
# The edtlib.EDT object in pickle format.
set(EDT_PICKLE                  ${PROJECT_BINARY_DIR}/edt.pickle)
# The generated file containing the final DTS, for debugging.
set(ZEPHYR_DTS                  ${PROJECT_BINARY_DIR}/zephyr.dts)
# The generated C header needed by <zephyr/devicetree.h>
set(DEVICETREE_GENERATED_H      ${BINARY_DIR_INCLUDE_GENERATED}/devicetree_generated.h)
# Generated build system internals.
set(DTS_POST_CPP                ${PROJECT_BINARY_DIR}/zephyr.dts.pre)
set(DTS_DEPS                    ${PROJECT_BINARY_DIR}/zephyr.dts.d)

# This generates DT information needed by the Kconfig APIs.
set(GEN_DRIVER_KCONFIG_SCRIPT   ${DT_SCRIPTS}/gen_driver_kconfig_dts.py)
# Generated Kconfig symbols go here.
set(DTS_KCONFIG                 ${KCONFIG_BINARY_DIR}/Kconfig.dts)

# This generates DT information needed by the CMake APIs.
set(GEN_DTS_CMAKE_SCRIPT        ${DT_SCRIPTS}/gen_dts_cmake.py)
# The generated information itself, which we include() after
# creating it.
set(DTS_CMAKE                   ${PROJECT_BINARY_DIR}/dts.cmake)

# The location of a file containing known vendor prefixes, relative to
# each element of DTS_ROOT. Users can define their own in their own
# modules.
set(VENDOR_PREFIXES             dts/bindings/vendor-prefixes.txt)

# The C preprocessor to use.
set_ifndef(CMAKE_DTS_PREPROCESSOR ${CMAKE_C_COMPILER})

#
# Halt execution early if there is no devicetree.
#

# TODO: What to do about non-posix platforms where NOT CONFIG_HAS_DTS (xtensa)?
# Drop support for NOT CONFIG_HAS_DTS perhaps?
set_ifndef(DTS_SOURCE ${BOARD_DIR}/${BOARD}.dts)
if(EXISTS ${DTS_SOURCE})
  # We found a devicetree. Check for a board revision overlay.
  if(BOARD_REVISION AND EXISTS ${BOARD_DIR}/${BOARD}_${BOARD_REVISION_STRING}.overlay)
    list(APPEND DTS_SOURCE ${BOARD_DIR}/${BOARD}_${BOARD_REVISION_STRING}.overlay)
  endif()
else()
  # If we don't have a devicetree after all, there's not much to do.
  set(header_template ${ZEPHYR_BASE}/misc/generated/generated_header.template)
  zephyr_file_copy(${header_template} ${DEVICETREE_GENERATED_H} ONLY_IF_DIFFERENT)
  return()
endif()

#
# Finalize the value of DTS_ROOT, so we know where all our
# DTS files, bindings, and vendor prefixes are.
#

# Convert relative paths to absolute paths relative to the application
# source directory.
zephyr_file(APPLICATION_ROOT DTS_ROOT)

# DTS_ROOT always includes the application directory, the board
# directory, shield directories, and ZEPHYR_BASE.
list(APPEND
  DTS_ROOT
  ${APPLICATION_SOURCE_DIR}
  ${BOARD_DIR}
  ${SHIELD_DIRS}
  ${ZEPHYR_BASE}
  )

# Convert the directories in DTS_ROOT to absolute paths without
# symlinks.
#
# DTS directories can come from multiple places. Some places, like a
# user's CMakeLists.txt can preserve symbolic links. Others, like
# scripts/zephyr_module.py --settings-out resolve them.
unset(real_dts_root)
foreach(dts_dir ${DTS_ROOT})
  file(REAL_PATH ${dts_dir} real_dts_dir)
  list(APPEND real_dts_root ${real_dts_dir})
endforeach()
set(DTS_ROOT ${real_dts_root})

# Finally, de-duplicate the list.
list(REMOVE_DUPLICATES
  DTS_ROOT
  )

#
# Find all the DTS files we need to concatenate and preprocess, as
# well as all the devicetree bindings and vendor prefixes associated
# with them.
#

set(dts_files
  ${DTS_SOURCE}
  ${shield_dts_files}
  )

if(DTC_OVERLAY_FILE)
  # Convert from space-separated files into file list
  string(CONFIGURE "${DTC_OVERLAY_FILE}" DTC_OVERLAY_FILE_EXPANDED)
  string(REPLACE " " ";" DTC_OVERLAY_FILE_RAW_LIST "${DTC_OVERLAY_FILE_EXPANDED}")
  foreach(file ${DTC_OVERLAY_FILE_RAW_LIST})
    file(TO_CMAKE_PATH "${file}" cmake_path_file)
    list(APPEND DTC_OVERLAY_FILE_AS_LIST ${cmake_path_file})
  endforeach()
  list(APPEND
    dts_files
    ${DTC_OVERLAY_FILE_AS_LIST}
    )
endif()

set(i 0)
unset(DTC_INCLUDE_FLAG_FOR_DTS)
foreach(dts_file ${dts_files})
  list(APPEND DTC_INCLUDE_FLAG_FOR_DTS
       -include ${dts_file})

  if(i EQUAL 0)
    message(STATUS "Found BOARD.dts: ${dts_file}")
  else()
    message(STATUS "Found devicetree overlay: ${dts_file}")
  endif()

  math(EXPR i "${i}+1")
endforeach()

unset(DTS_ROOT_SYSTEM_INCLUDE_DIRS)
unset(DTS_ROOT_BINDINGS)
foreach(dts_root ${DTS_ROOT})
  foreach(dts_root_path
      include
      include/zephyr
      dts/common
      dts/${ARCH}
      dts
      )
    get_filename_component(full_path ${dts_root}/${dts_root_path} REALPATH)
    if(EXISTS ${full_path})
      list(APPEND
        DTS_ROOT_SYSTEM_INCLUDE_DIRS
        -isystem ${full_path}
        )
    endif()
  endforeach()

  set(bindings_path ${dts_root}/dts/bindings)
  if(EXISTS ${bindings_path})
    list(APPEND
      DTS_ROOT_BINDINGS
      ${bindings_path}
      )
  endif()

  set(vendor_prefixes ${dts_root}/${VENDOR_PREFIXES})
  if(EXISTS ${vendor_prefixes})
    list(APPEND EXTRA_GEN_DEFINES_ARGS --vendor-prefixes ${vendor_prefixes})
  endif()
endforeach()

# Cache the location of the root bindings so they can be used by
# scripts which use the build directory.
set(CACHED_DTS_ROOT_BINDINGS ${DTS_ROOT_BINDINGS} CACHE INTERNAL
  "DT bindings root directories")

#
# Run the C preprocessor on the devicetree source, so we can parse it
# (using the Python devicetree package) in later steps.
#

# TODO: Cut down on CMake configuration time by avoiding
# regeneration of devicetree_generated.h on every configure. How
# challenging is this? Can we cache the dts dependencies?

# Run the preprocessor on the DTS input files. We are leaving
# linemarker directives enabled on purpose. This tells dtlib where
# each line actually came from, which improves error reporting.
execute_process(
  COMMAND ${CMAKE_DTS_PREPROCESSOR}
  -x assembler-with-cpp
  -nostdinc
  ${DTS_ROOT_SYSTEM_INCLUDE_DIRS}
  ${DTC_INCLUDE_FLAG_FOR_DTS}  # include the DTS source and overlays
  ${NOSYSDEF_CFLAG}
  -D__DTS__
  ${DTS_EXTRA_CPPFLAGS}
  -E   # Stop after preprocessing
  -MD  # Generate a dependency file as a side-effect
  -MF ${DTS_DEPS}
  -o ${DTS_POST_CPP}
  ${ZEPHYR_BASE}/misc/empty_file.c
  WORKING_DIRECTORY ${APPLICATION_SOURCE_DIR}
  RESULT_VARIABLE ret
  )
if(NOT "${ret}" STREQUAL "0")
  message(FATAL_ERROR "command failed with return code: ${ret}")
endif()

#
# Make sure we re-run CMake if any devicetree sources or transitive
# includes change.
#

# Parse the generated dependency file to find the DT sources that
# were included, including any transitive includes.
toolchain_parse_make_rule(${DTS_DEPS}
  include_files # Output parameter
  )

# Add the results to the list of files that, when change, force the
# build system to re-run CMake.
set_property(DIRECTORY APPEND PROPERTY
  CMAKE_CONFIGURE_DEPENDS
  ${include_files}
  ${GEN_DEFINES_SCRIPT}
  ${GEN_DRIVER_KCONFIG_SCRIPT}
  ${GEN_DTS_CMAKE_SCRIPT}
  )

#
# Run GEN_DEFINES_SCRIPT.
#

string(REPLACE ";" " " EXTRA_DTC_FLAGS_RAW "${EXTRA_DTC_FLAGS}")
set(CMD_GEN_DEFINES ${PYTHON_EXECUTABLE} ${GEN_DEFINES_SCRIPT}
--dts ${DTS_POST_CPP}
--dtc-flags '${EXTRA_DTC_FLAGS_RAW}'
--bindings-dirs ${DTS_ROOT_BINDINGS}
--header-out ${DEVICETREE_GENERATED_H}.new
--dts-out ${ZEPHYR_DTS}.new # for debugging and dtc
--edt-pickle-out ${EDT_PICKLE}
${EXTRA_GEN_DEFINES_ARGS}
)

execute_process(
  COMMAND ${CMD_GEN_DEFINES}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  RESULT_VARIABLE ret
  )
if(NOT "${ret}" STREQUAL "0")
  message(STATUS "In: ${PROJECT_BINARY_DIR}, command: ${CMD_GEN_DEFINES}")
  message(FATAL_ERROR "gen_defines.py failed with return code: ${ret}")
else()
  zephyr_file_copy(${ZEPHYR_DTS}.new ${ZEPHYR_DTS} ONLY_IF_DIFFERENT)
  zephyr_file_copy(${DEVICETREE_GENERATED_H}.new ${DEVICETREE_GENERATED_H} ONLY_IF_DIFFERENT)
  file(REMOVE ${ZEPHYR_DTS}.new ${DEVICETREE_GENERATED_H}.new)
  message(STATUS "Generated zephyr.dts: ${ZEPHYR_DTS}")
  message(STATUS "Generated devicetree_generated.h: ${DEVICETREE_GENERATED_H}")
endif()

#
# Run GEN_DRIVER_KCONFIG_SCRIPT.
#

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${GEN_DRIVER_KCONFIG_SCRIPT}
  --kconfig-out ${DTS_KCONFIG}
  --bindings-dirs ${DTS_ROOT_BINDINGS}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  RESULT_VARIABLE ret
  )
if(NOT "${ret}" STREQUAL "0")
  message(FATAL_ERROR "gen_driver_kconfig_dts.py failed with return code: ${ret}")
endif()

#
# Run GEN_DTS_CMAKE_SCRIPT.
#

execute_process(
  COMMAND ${PYTHON_EXECUTABLE} ${GEN_DTS_CMAKE_SCRIPT}
  --edt-pickle ${EDT_PICKLE}
  --cmake-out ${DTS_CMAKE}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  RESULT_VARIABLE ret
  )
if(NOT "${ret}" STREQUAL "0")
  message(FATAL_ERROR "gen_dts_cmake.py failed with return code: ${ret}")
else()
  message(STATUS "Including generated dts.cmake file: ${DTS_CMAKE}")
  include(${DTS_CMAKE})
endif()

#
# Run dtc if it was found.
#
# This is just to generate warnings and errors; we discard the output.
#

if(DTC)

set(DTC_WARN_UNIT_ADDR_IF_ENABLED "")
check_dtc_flag("-Wunique_unit_address_if_enabled" check)
if (check)
  set(DTC_WARN_UNIT_ADDR_IF_ENABLED "-Wunique_unit_address_if_enabled")
endif()

set(DTC_NO_WARN_UNIT_ADDR "")
check_dtc_flag("-Wno-unique_unit_address" check)
if (check)
  set(DTC_NO_WARN_UNIT_ADDR "-Wno-unique_unit_address")
endif()

set(VALID_EXTRA_DTC_FLAGS "")
foreach(extra_opt ${EXTRA_DTC_FLAGS})
  check_dtc_flag(${extra_opt} check)
  if (check)
    list(APPEND VALID_EXTRA_DTC_FLAGS ${extra_opt})
  endif()
endforeach()
set(EXTRA_DTC_FLAGS ${VALID_EXTRA_DTC_FLAGS})

execute_process(
  COMMAND ${DTC}
  -O dts
  -o - # Write output to stdout, which we discard below
  -b 0
  -E unit_address_vs_reg
  ${DTC_NO_WARN_UNIT_ADDR}
  ${DTC_WARN_UNIT_ADDR_IF_ENABLED}
  ${EXTRA_DTC_FLAGS} # User settable
  ${ZEPHYR_DTS}
  OUTPUT_QUIET # Discard stdout
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  RESULT_VARIABLE ret
  )

if(NOT "${ret}" STREQUAL "0")
  message(FATAL_ERROR "command failed with return code: ${ret}")
endif()
endif(DTC)
