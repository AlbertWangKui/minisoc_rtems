#!/bin/bash

# native_build.sh
#
# This script is designed to run inside a pre-configured build environment
# (like the rtems_lite:v0.3 Docker container). It directly invokes Waf
# without any Docker wrappers. It accepts the same arguments as docker_build.sh.

# --- Default values ---
WAF_COMMAND="./waf"
JOBS=16
CLEAN_BUILD=0
WAF_CONFIGURE_ARGS=""

# --- Function to display help message ---
usage() {
    echo "Usage: $0 [options]"
    echo "This script builds the project inside a pre-configured environment (e.g., a Docker container)."
    echo ""
    echo "Options:"
    echo "  --prj-name=NAME          Project name (e.g., tianhe, sheshou)"
    echo "  --prj-platform=PLATFORM  Platform type (e.g., fpga, emu, asic)"
    echo "  --prj-cflags=CFLAGS      Project specific CFLAGS"
    echo "  --prj-ldflags=LDFLAGS    Project specific LDFLAGS"
    echo "  --prj-linkflags=LINKFLAGS Project specific LINKFLAGS"
    echo "  --prj-defconfig=FILE     Use custom defconfig file"
    echo "  -c, --clean              Run 'waf clean'"
    echo "  -j, --jobs=NUM           Number of parallel jobs for waf build (default: 16)"
    echo "  -h, --help               Display this help message"
    echo ""
    echo "Examples:"
    echo "  ./native_build.sh --prj-name=tianhe --prj-platform=fpga"
    echo "  ./native_build.sh -c"
    echo "  ./native_build.sh --prj-name=tianhe --prj-platform=fpga -j8"
}

# --- Parse command-line arguments ---
# Forward all arguments that start with --prj- to waf configure
for arg in "$@"; do
    case $arg in
        --prj-*)
        WAF_CONFIGURE_ARGS="$WAF_CONFIGURE_ARGS $arg"
        ;;
        -c|--clean)
        CLEAN_BUILD=1
        ;;
        -j=*|--jobs=*)
        JOBS="${arg#*=}"
        ;;
        -h|--help)
        usage
        exit 0
        ;;
        *)
        # Silently ignore unknown options
        ;;
    esac
done

# --- Main build LOGEc ---
echo "Starting native build..."
echo "Arguments passed to waf configure: ${WAF_CONFIGURE_ARGS}"
echo "Parallel jobs: ${JOBS}"

# Handle 'clean' command
if [ "$CLEAN_BUILD" -eq 1 ]; then
    echo "Executing: $WAF_COMMAND clean"
    $WAF_COMMAND clean
    exit $?
fi

# Execute configure and build steps
BUILD_COMMAND="$WAF_COMMAND configure ${WAF_CONFIGURE_ARGS} && $WAF_COMMAND build -j${JOBS}"

echo "-------------------------------------------"
echo "Build Command : ${BUILD_COMMAND}"
echo "-------------------------------------------"

# Execute the command
eval ${BUILD_COMMAND}