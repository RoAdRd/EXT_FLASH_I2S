# Install script for directory: /opt/nordic/ncs/v2.9.0/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/opt/nordic/ncs/toolchains/b8efef2ad5/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/nrf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/cjson/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/azure-sdk-for-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/cirrus-logic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/suit-processor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/memfault-firmware-sdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/canopennode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/chre/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/nrf_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/nrfxlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/modules/connectedhomeip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/opt/nordic/ncs/v2.9.0/zephyr/samples/drivers/spi_flash/build/spi_flash/zephyr/cmake/reports/cmake_install.cmake")
endif()

