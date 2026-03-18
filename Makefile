SHELL := /bin/bash

############################################################
# Standard Configuration
############################################################
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
repo_path := $(dir $(mkfile_path))
biped_wb_mpc_path := $(abspath $(repo_path)/biped_wb_mpc)
build_dir ?= $(abspath $(repo_path)/../..)

CCACHE_DIR := $(build_dir)/.ccache

ros_source_file := /bin/ros_setup.sh

ifeq ("$(wildcard /opt/ros/jazzy/setup.bash)","")
    ifeq ("$(wildcard $(ros_source_file))","")
        ros_source_file := /opt/ros/humble/setup.bash
    endif
else
    ifeq ("$(wildcard $(ros_source_file))","")
        ros_source_file := /opt/ros/jazzy/setup.bash
    endif
endif

LINKER_FLAGS = "$(shell python3-config --ldflags --embed)"

# Find ROS2 packages in a given directory, two levels deep, and return only the package name
define find_ros2_packages
$(shell \
    for dir in $$(find $(1) -mindepth 1 -maxdepth 2 -type d); do \
        if [ -f "$$dir/CMakeLists.txt" ] && [ -f "$$dir/package.xml" ]; then \
            basename $$dir; \
        fi; \
    done)
endef

# Individual package lists from specific subdirectories in biped_wb_mpc
NMPC_PACKAGES := $(call find_ros2_packages,$(biped_wb_mpc_path)/humanoid_nmpc)
ROBOT_MODEL_PACKAGES := $(call find_ros2_packages,$(biped_wb_mpc_path)/robot_models)
RUNTIME_PACKAGES := $(call find_ros2_packages,$(biped_wb_mpc_path)/robot_runtime)

# Unified package list: build only biped_wb_mpc packages by default
PACKAGES ?= $(NMPC_PACKAGES) $(ROBOT_MODEL_PACKAGES) $(RUNTIME_PACKAGES)

############################################################
# Customizable Configuration - User can override these
############################################################
BUILD_TYPE ?= Release
BUILD_TESTING ?= ON
BUILD_WITH_NINJA ?= ON
PARALLEL_JOBS ?= 6
CPP_VERSION ?= -std=c++20

############################################################
# Set flags based on configuration
############################################################
COMMON_CMAKE_ARGS ?= \
	-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	-DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
	-DBUILD_TESTING=$(BUILD_TESTING) \
	-DCMAKE_SHARED_LINKER_FLAGS=$(LINKER_FLAGS) \
	-DCMAKE_CXX_FLAGS=$(CPP_VERSION)

ifeq ($(BUILD_WITH_NINJA), ON)
	BUILD_SYSTEM=Ninja
	EVENT_HANDLERS=--event-handlers=console_cohesion+
	COMMON_CMAKE_ARGS += \
	-G${BUILD_SYSTEM} \
	-DCMAKE_C_COMPILER_LAUNCHER=ccache \
	-DCMAKE_CXX_COMPILER_LAUNCHER=ccache
else
	BUILD_SYSTEM="Unix Makefiles"
	COMMON_CMAKE_ARGS += \
	-G${BUILD_SYSTEM}
endif

COMMON_COLCON_BUILD_FLAGS ?= \
	--parallel-workers=${PARALLEL_JOBS} \
	${EVENT_HANDLERS} \
	--symlink-install \
	--build-base $(build_dir)/build \
	--install-base $(build_dir)/install

############################################################
# Define build and test targets
############################################################
define default-build-package
	cd ${build_dir} && \
	export MAKEFLAGS="-j ${PARALLEL_JOBS} -d" && \
	source ${ros_source_file} && \
	colcon build ${COMMON_COLCON_BUILD_FLAGS} --packages-up-to $(1) \
	--cmake-args ${COMMON_CMAKE_ARGS} $(EXTRA_CMAKE_ARGS) && \
	source $(build_dir)/install/setup.bash
endef

define default-test-package
  cd ${build_dir} && \
  source ${ros_source_file} && \
  source $(build_dir)/install/setup.bash && \
  colcon test --packages-select $(1) --event-handlers console_direct+ --return-code-on-test-failure
endef

############################################################
# Command Line Interface
############################################################
.PHONY: build-biped_wb_mpc build-debug build-release build-relwithdebinfo build \
        test-all test $(addprefix build-,$(PACKAGES)) $(addprefix test-,$(PACKAGES)) \
        echo-packages clean-ws

# Builds only packages found under biped_wb_mpc/
build-biped_wb_mpc:
	$(call default-build-package,$(PACKAGES))

$(addprefix build-,$(PACKAGES)):
	$(call default-build-package,$(patsubst build-%,%,$@))

build:
	@$(if $(PKG),$(call default-build-package,$(PKG)),@echo "Please specify a package to build by setting the PKG variable. Example: make build PKG=package_name")

build-debug:
	@$(MAKE) BUILD_TYPE=Debug $(if $(PKG),build PKG=$(PKG),build-biped_wb_mpc)

build-release:
	@$(MAKE) BUILD_TYPE=Release BUILD_TESTING=OFF $(if $(PKG),build PKG=$(PKG),build-biped_wb_mpc)

build-relwithdebinfo:
	@$(MAKE) BUILD_TYPE=RelWithDebInfo $(if $(PKG),build PKG=$(PKG),build-biped_wb_mpc)

test-all: $(addprefix test-,$(PACKAGES))

$(addprefix test-,$(PACKAGES)):
	$(call default-test-package,$(patsubst test-%,%,$@))

test:
	@$(if $(PKG),$(call default-test-package,$(PKG)),@echo "Please specify a package to test by setting the PKG variable. Example: make test PKG=package_name")

echo-packages:
	@echo "Packages to be built from biped_wb_mpc: $(PACKAGES)"

clean-ws:
	cd ${build_dir} && \
	rm -rf build install log .ccache
