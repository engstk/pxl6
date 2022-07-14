#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

EXPERIMENTAL_BUILD=${EXPERIMENTAL_BUILD:-0}
TRIM_NONLISTED_KMI=${TRIM_NONLISTED_KMI:-1}
LTO=${LTO:-thin}
KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI:-1}
DEFAULT_CONFIG="private/gs-google/build.config.slider"
DEVICE_KERNEL_BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG:-${DEFAULT_CONFIG}}
GKI_KERNEL_PREBUILTS_DIR=
GKI_KERNEL_BUILD_CONFIG=
GKI_KERNEL_OUT_DIR=
CHECK_DIRTY_AOSP=0
if [ -z "${BUILD_KERNEL}" ]; then
  if [ "${EXPERIMENTAL_BUILD}" != "0" -o -n "${GKI_DEFCONFIG_FRAGMENT}" ]; then
    BUILD_KERNEL=1
  else
    CHECK_DIRTY_AOSP=1
    BUILD_KERNEL=0
  fi
fi

if [ "${BUILD_KERNEL}" = "0" ]; then
  USING_PREBUILTS=1
  GKI_KERNEL_PREBUILTS_DIR=$(readlink -m "prebuilts/boot-artifacts/kernel/")
else
  USING_PREBUILTS=
  if [ "${EXPERIMENTAL_BUILD}" != "0" ]; then
    GKI_KERNEL_OUT_DIR=android13-5.10-staging
    GKI_KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64
  else
    GKI_KERNEL_OUT_DIR=android13-5.10
    GKI_KERNEL_BUILD_CONFIG=aosp/build.config.gki.aarch64
  fi
fi

if [ "${LTO}" = "none" ]; then
  echo "LTO=none requires disabling KMI_SYMBOL_STRICT_MODE. Setting to 0..."
  KMI_SYMBOL_LIST_STRICT_MODE=0
fi

if [ -n "${BUILD_ABI}" ]; then
  echo "The ABI update workflow has changed. Please read go/gki-p21-workflow"
  echo "  for instructions on updating ABI/symbol list."
  exit_if_error 1 "BUILD_ABI is deprecated"
fi

if [ "${BUILD_KERNEL}" = "0" ]; then
  if [ "${LTO}" = "none" ]; then
    echo "LTO=none requires BUILD_KERNEL=1, EXPERIMENTAL_BUILD=1, or"
    echo "  GKI_DEFCONFIG_FRAGMENT to be set."
    exit_if_error 1 "LTO=none requires building the kernel"
  elif [ -n "${GKI_DEFCONFIG_FRAGMENT}" -o \
            "${EXPERIMENTAL_BUILD}" != "0" ]; then
    echo "BUILD_KERNEL=0 is incompatible with EXPERIMENTAL_BUILD and"
    echo "  GKI_DEFCONFIG_FRAGMENT."
    exit_if_error 1 "Flags incompatible with BUILD_KERNEL detected"
  fi
fi

# These are for build.sh, so they should be exported.
export LTO
export KMI_SYMBOL_LIST_STRICT_MODE
export TRIM_NONLISTED_KMI
export BASE_OUT=${OUT_DIR:-out}/mixed/
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
export USING_PREBUILTS

DEVICE_KERNEL_BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG} \
  GKI_KERNEL_BUILD_CONFIG=${GKI_KERNEL_BUILD_CONFIG} \
  GKI_KERNEL_OUT_DIR=${GKI_KERNEL_OUT_DIR} \
  GKI_KERNEL_PREBUILTS_DIR=${GKI_KERNEL_PREBUILTS_DIR} \
  GKI_DEFCONFIG_FRAGMENT=${GKI_DEFCONFIG_FRAGMENT} \
  ./build_mixed.sh "$@"

exit_if_error $? "Failed to create mixed build"

if [ -f ${GKI_KERNEL_PREBUILTS_DIR}/vmlinux ]; then
  SHA_FILE=vmlinux
else
  SHA_FILE=boot.img
fi

# If BUILD_KERNEL is not explicitly set, be sure that there are no aosp/
# changes not present in the prebuilt.
if [ "${CHECK_DIRTY_AOSP}" != "0" ]; then
  PREBUILTS_SHA=$(strings ${GKI_KERNEL_PREBUILTS_DIR}/${SHA_FILE} |
                     grep "Linux version 5.10" |
                     sed -n "s/^.*-g\([0-9a-f]\{12\}\)-.*/\1/p")
  pushd aosp/ > /dev/null
    # The AOSP sha can sometimes be longer than 12 characters; fix its length.
    AOSP_SHA=$(git log -1 --abbrev=12 --pretty="format:%h")
    if [ "${PREBUILTS_SHA}" != "${AOSP_SHA}" -o -n \
         "$(git --no-optional-locks status -uno --porcelain ||
            git diff-index --name-only HEAD)" ]; then
      echo "WARNING: There are aosp/ changes which are not in the prebuilts."
      echo "  Because you did not specify BUILD_KERNEL=0 or 1, $0"
      echo "  defaulted to building with the prebuilts. Please be aware that"
      echo "  your changes to aosp/ will not be present in the final images. If"
      echo "  you have made changes to aosp/, it is recommended to explicitly"
      echo "  set BUILD_KERNEL=0 if you wish to use the prebuilts, or to 1 if"
      echo "  you wish to build any local changes you may have."
    fi
  popd > /dev/null
fi
