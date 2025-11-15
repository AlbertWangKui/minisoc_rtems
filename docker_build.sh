#!/bin/bash

# WAF Build Script with Docker - Enhanced with compilation parameter support
# Usage:
#        ./docker_build.sh                                          # Auto-detect parameters and build (类似Makefile的make)
#        ./docker_build.sh --prj-name=sheshou --prj-platform=asic   # Build with specific parameters (类似Makefile的make PRJ_NAME=xxx PRJ_PLATFORM=xxx)  
#        ./docker_build.sh --prj-name=sheshou --prj-platform=asic --prj-cflags='-O2 -g'  # With custom CFLAGS (类似Makefile的make PRJ_CFLAGS=xxx)
#        ./docker_build.sh --prj-name=sheshou --prj-platform=asic --prj-ldflags='-Wl,--gc-sections'  # With custom LDFLAGS
#        ./docker_build.sh --prj-name=sheshou --prj-platform=asic --prj-linkflags='--specs=custom.specs'  # With custom LINKFLAGS
#        ./docker_build.sh -c "./waf clean"                         # Execute custom WAF command
#        ./docker_build.sh -c "./waf menuconfig"                    # Run menuconfig
#
# All parameter detection and validation is handled by WAF (in wscript), just like Makefile
# import docker image and build by docker image, docker image have bsp build env && toolchain

NC='\033[0m'

# Regular Colors
BLACK='\033[0;30m'        # Black
RED='\033[0;31m'          # Red
GREEN='\033[0;32m'        # Green
YELLOW='\033[0;33m'       # Yellow
BLUE='\033[0;34m'         # Blue
PURPLE='\033[0;35m'       # Purple
CYAN='\033[0;36m'         # Cyan
WHITE='\033[0;37m'        # White

# Bold
BBLACK='\033[1;30m'       # Black
BRED='\033[1;31m'         # Red
BGREEN='\033[1;32m'       # Green
BYELLOW='\033[1;33m'      # Yellow
BBLUE='\033[1;34m'        # Blue
BPURPLE='\033[1;35m'      # Purple
BCYAN='\033[1;36m'        # Cyan
BWHITE='\033[1;37m'       # White

#should confirm, this keywords maybe changed for different proj
#IMAGE_KEYWORDS=pub-rtems
IMAGE_KEYWORDS=rtems_lite
IMAGE_TAG=v0.3

# 指定镜像ID 全部代码版本使用相同docker镜像
IMAGE_ID=""
IMAGE_NAME=

exec 3>&2
#exec 2> /dev/null

# Parse command line arguments
COMMAND=""
PRJ_NAME=""
PRJ_PLATFORM=""
PRJ_DEFCONFIG=""
PRJ_CFLAGS=""
PRJ_LDFLAGS=""
PRJ_LINKFLAGS=""

# Parse all arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c)
            COMMAND="$2"
            shift 2
            ;;
        --prj-name=*)
            PRJ_NAME="${1#*=}"
            shift
            ;;
        --prj-platform=*)
            PRJ_PLATFORM="${1#*=}"
            shift
            ;;
        --prj-defconfig=*)
            PRJ_DEFCONFIG="${1#*=}"
            shift
            ;;
        --prj-cflags=*)
            PRJ_CFLAGS="${1#*=}"
            shift
            ;;
        --prj-ldflags=*)
            PRJ_LDFLAGS="${1#*=}"
            shift
            ;;
        --prj-linkflags=*)
            PRJ_LINKFLAGS="${1#*=}"
            shift
            ;;
        *)
            echo -e "${BRED}Error: Unknown argument '$1'${NC}"
            echo -e "Usage: ./docker_build.sh [--prj-name=PROJECT] [--prj-platform=PLATFORM] [--prj-defconfig=FILE] [--prj-cflags=FLAGS] [--prj-ldflags=FLAGS] [--prj-linkflags=FLAGS] [-c COMMAND]"
            echo -e "Examples:"
            echo -e "  ./docker_build.sh                                          # Auto-detect"
            echo -e "  ./docker_build.sh --prj-name=sheshou --prj-platform=asic   # With parameters"
            echo -e "  ./docker_build.sh --prj-name=sheshou --prj-platform=asic --prj-defconfig=custom.config  # With custom defconfig"
            echo -e "  ./docker_build.sh --prj-name=sheshou --prj-platform=asic --prj-cflags='-O2 -g'  # With custom CFLAGS"
            echo -e "  ./docker_build.sh -c './waf clean'                         # Custom command"
            exit 1
            ;;
    esac
done

# Build command based on arguments - WAF handles all parameter logic
if [[ -z $COMMAND ]];then
    if [[ -n $PRJ_NAME && -n $PRJ_PLATFORM ]]; then
        # Both required parameters provided - build configure command with all optional parameters
        CONFIGURE_CMD="./waf configure --prj-name=$PRJ_NAME --prj-platform=$PRJ_PLATFORM"
        
        # Add optional parameters if provided
        if [[ -n $PRJ_DEFCONFIG ]]; then
            CONFIGURE_CMD="$CONFIGURE_CMD --prj-defconfig=$PRJ_DEFCONFIG"
        fi
        if [[ -n $PRJ_CFLAGS ]]; then
            CONFIGURE_CMD="$CONFIGURE_CMD --prj-cflags='$PRJ_CFLAGS'"
        fi
        if [[ -n $PRJ_LDFLAGS ]]; then
            CONFIGURE_CMD="$CONFIGURE_CMD --prj-ldflags='$PRJ_LDFLAGS'"
        fi
        if [[ -n $PRJ_LINKFLAGS ]]; then
            CONFIGURE_CMD="$CONFIGURE_CMD --prj-linkflags='$PRJ_LINKFLAGS'"
        fi
        
        COMMAND="$CONFIGURE_CMD && ./waf build -j16"
        
        # Display information about what parameters are being used
        echo -e "${BYELLOW}Building with specified parameters:${NC}"
        echo -e "  --prj-name=${BGREEN}$PRJ_NAME${NC} --prj-platform=${BGREEN}$PRJ_PLATFORM${NC}"
        if [[ -n $PRJ_DEFCONFIG ]]; then
            echo -e "  --prj-defconfig=${BGREEN}$PRJ_DEFCONFIG${NC}"
        fi
        if [[ -n $PRJ_CFLAGS ]]; then
            echo -e "  --prj-cflags=${BGREEN}'$PRJ_CFLAGS'${NC}"
        fi
        if [[ -n $PRJ_LDFLAGS ]]; then
            echo -e "  --prj-ldflags=${BGREEN}'$PRJ_LDFLAGS'${NC}"
        fi
        if [[ -n $PRJ_LINKFLAGS ]]; then
            echo -e "  --prj-linkflags=${BGREEN}'$PRJ_LINKFLAGS'${NC}"
        fi
    elif [[ -n $PRJ_NAME || -n $PRJ_PLATFORM || -n $PRJ_DEFCONFIG || -n $PRJ_CFLAGS || -n $PRJ_LDFLAGS || -n $PRJ_LINKFLAGS ]]; then
        # Some parameters provided but not both required ones - this is an error
        echo -e "${BRED}Error: Both --prj-name and --prj-platform must be specified together${NC}"
        echo -e "Optional parameters: --prj-defconfig, --prj-cflags, --prj-ldflags, --prj-linkflags"
        echo -e "Usage:"
        echo -e "  ./docker_build.sh                                          # Auto-detect parameters"
        echo -e "  ./docker_build.sh --prj-name=PROJECT --prj-platform=PLATFORM # Both parameters required"
        echo -e "  ./docker_build.sh --prj-name=PROJECT --prj-platform=PLATFORM --prj-defconfig=FILE # With custom defconfig"
        echo -e "  ./docker_build.sh --prj-name=PROJECT --prj-platform=PLATFORM --prj-cflags='-O2 -g' # With custom CFLAGS"
        echo -e ""
        params_provided=""
        [[ -n $PRJ_NAME ]] && params_provided="$params_provided--prj-name=$PRJ_NAME "
        [[ -n $PRJ_PLATFORM ]] && params_provided="$params_provided--prj-platform=$PRJ_PLATFORM "
        [[ -n $PRJ_DEFCONFIG ]] && params_provided="$params_provided--prj-defconfig=$PRJ_DEFCONFIG "
        [[ -n $PRJ_CFLAGS ]] && params_provided="$params_provided--prj-cflags='$PRJ_CFLAGS' "
        [[ -n $PRJ_LDFLAGS ]] && params_provided="$params_provided--prj-ldflags='$PRJ_LDFLAGS' "
        [[ -n $PRJ_LINKFLAGS ]] && params_provided="$params_provided--prj-linkflags='$PRJ_LINKFLAGS' "
        echo -e "Provided: $params_provided"
        exit 1
    else
        # No parameters - WAF will auto-detect (like Makefile's 'make')
        COMMAND="./waf configure && ./waf build -j16"  
        echo -e "${BYELLOW}Building with auto-detection mode (like Makefile without parameters)${NC}"
        echo -e "${CYAN}WAF will automatically detect project parameters from existing configuration${NC}"
    fi
fi

echo -n "Checking if Docker is installed...  "

if docker --version | grep "Docker version" > /dev/null; then
    echo -e "${CYAN}pass${NC}"
else
    echo -e "${RED}failed${NC}"
    exit
fi

IMAGE_ID=`docker images | grep $IMAGE_KEYWORDS | grep $IMAGE_TAG | awk '{print $3}'`

echo -n "Checking if Image is exist...   "
IMAGE_NAMES=$(docker images --format "table {{.ID}}\t{{.Repository}}:{{.Tag}}" | grep $IMAGE_ID | awk '{print $2}')
if [[ -z $IMAGE_NAMES ]];then
    echo -e "${RED}failed${NC}"
    echo "Bug: Docker Image ID $IMAGE_ID not exist, please check docker env"
    exit
else
    echo -e "${CYAN}pass${NC}"
fi

echo -e "-------------------------------------------"
echo -e "Build Command : ${BRED}$COMMAND${NC}"
INDEX=0
for IMAGE_NAME in $IMAGE_NAMES; do
    echo -e "Image Name[$INDEX] : ${BGREEN}$IMAGE_NAME${NC}"
    INDEX=`expr $INDEX + 1`
done
echo -e "Image ID      : ${BBLUE}$IMAGE_ID${NC}"
echo -e "-------------------------------------------"

echo "Exec command in docker..."
echo -e "-------------------------------------------"


#docker run --rm -it \
 #    -e DOCKER_IMAGE_NAME=$IMAGE_NAME  \
 #    -e DOCKER_IMAGE_ID=$IMAGE_ID \
 #    --user $(id -u):$(id -g) \
 #    -v /etc/group:/etc/group:ro \
 #    -v /etc/passwd:/etc/passwd:ro  \
 #    -v $(pwd):/host \
 #    --workdir /host \
 #    $IMAGE_ID \
 #    /bin/bash -c "$COMMAND"
#docker run --rm -it \

docker run --rm --cpus=2 --memory=4g \
    -e DOCKER_IMAGE_NAME=$IMAGE_NAME  \
    -e DOCKER_IMAGE_ID=$IMAGE_ID \
    -v $(pwd):$(pwd) \
    --workdir  $(pwd) \
    $IMAGE_ID \
    /bin/bash -lc "$COMMAND"

echo -e "-------------------------------------------"

