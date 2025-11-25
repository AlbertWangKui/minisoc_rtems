#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
WAF build script for MiniSoC firmware project
Compatible with older Python versions and supports automatic parameter detection
"""

import os
import sys
import glob
from waflib import Logs, Utils, Options, Context


APPNAME = 'minisoc'
VERSION = '1.0.0'

top = '.'
out = 'build'

# Supported platforms and projects
SUPPORTED_PROJECTS = ['tianhe']
SUPPORTED_PLATFORMS = ['fpga', 'emu', 'asic']

def auto_detect_project_params():
    """
    Auto-detect project parameters from existing .config file
    Returns (prj_name, prj_platform, found_config)
    If .config file is not found, returns (None, None, None) to indicate failure
    """
    # Only try to detect from existing .config file
    config_file = '.config'
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                content = f.read()
                
                prj_name = None
                prj_platform = None
                
                # Look for BSP name and platform in config
                for line in content.split('\n'):
                    line = line.strip()
                    
                    # Method 1: Direct BSP name
                    if line.startswith('CONFIG_BSP_NAME='):
                        bsp_name = line.split('=')[1].strip('"')
                        if bsp_name in SUPPORTED_PROJECTS:
                            prj_name = bsp_name
                    
                    # Method 2: BSP selection (CONFIG_BSP_SHESHOU=y)
                    elif '=y' in line and line.startswith('CONFIG_BSP_'):
                        bsp_name = line.replace('CONFIG_BSP_', '').replace('=y', '').lower()
                        if bsp_name in SUPPORTED_PROJECTS:
                            prj_name = bsp_name
                    
                    # Platform detection (CONFIG_PLATFORM_FPGA=y)
                    elif '=y' in line and line.startswith('CONFIG_PLATFORM_'):
                        platform_name = line.replace('CONFIG_PLATFORM_', '').replace('=y', '').lower()
                        if platform_name in SUPPORTED_PLATFORMS:
                            prj_platform = platform_name
                
                # If both found, return success
                if prj_name and prj_platform:
                    return prj_name, prj_platform, "%s_%s_defconfig" % (prj_name, prj_platform)
                        
        except Exception as e:
            pass
    
    # If .config file doesn't exist or parsing failed, return failure
    # No longer try to analyze configs/ directory - user must explicitly configure
    return None, None, None

def options(opt):
    """Configure command line options"""
    opt.load('compiler_c')
    
    # Add custom project options
    opt.add_option('--prj-name', action='store', default=None,
                   help='Project name: %s (auto-detect if not specified)' % ', '.join(SUPPORTED_PROJECTS))
    opt.add_option('--prj-platform', action='store', default=None,
                   help='Platform type: %s (auto-detect if not specified)' % ', '.join(SUPPORTED_PLATFORMS))
    opt.add_option('--prj-cflags', action='store', default='',
                   help='Project specific CFLAGS (completely replaces default CFLAGS if provided)')
    opt.add_option('--prj-ldflags', action='store', default='',
                   help='Project specific LDFLAGS (completely replaces default LDFLAGS if provided)')
    opt.add_option('--prj-linkflags', action='store', default='',
                   help='Project specific LINKFLAGS (completely replaces default LINKFLAGS if provided)')
    opt.add_option('--prj-defconfig', action='store', default=None,
                   help='Use custom defconfig file')

def configure(ctx):
    """Configure the build environment"""
    ctx.load('compiler_c')
    
    # Get parameters from command line or auto-detect
    prj_name = getattr(Options.options, 'prj_name', None)
    prj_platform = getattr(Options.options, 'prj_platform', None)
    prj_cflags = getattr(Options.options, 'prj_cflags', '')
    prj_ldflags = getattr(Options.options, 'prj_ldflags', '')
    prj_linkflags = getattr(Options.options, 'prj_linkflags', '')
    prj_defconfig = getattr(Options.options, 'prj_defconfig', None)
    
    # Handle parameter validation
    cmd_line_params_provided = prj_name is not None or prj_platform is not None or prj_defconfig is not None
    
    if cmd_line_params_provided:
        # If any parameter is provided via command line, prj_name and prj_platform must be provided
        if prj_name is None or prj_platform is None:
            Logs.error('=== Parameter Error ===')
            Logs.error('When specifying parameters, both --prj-name and --prj-platform must be provided')
            Logs.error('Optional parameter: --prj-defconfig (if not specified, uses existing .config)')
            Logs.error('Usage:')
            Logs.error('  ./waf configure                                    # Auto-detect (like make)')
            Logs.error('  ./waf configure --prj-name=PROJECT --prj-platform=PLATFORM  # Use existing .config')
            Logs.error('  ./waf configure --prj-name=PROJECT --prj-platform=PLATFORM --prj-defconfig=FILE  # Use specified defconfig')
            Logs.error('')
            Logs.error('Supported projects: %s' % ', '.join(SUPPORTED_PROJECTS))
            Logs.error('Supported platforms: %s' % ', '.join(SUPPORTED_PLATFORMS))
            ctx.fatal('Missing required parameter')
        
        # Validate provided parameters
        if prj_name not in SUPPORTED_PROJECTS:
            ctx.fatal('Unsupported PRJ_NAME: %s. Supported: %s' % (prj_name, ', '.join(SUPPORTED_PROJECTS)))
        if prj_platform not in SUPPORTED_PLATFORMS:
            ctx.fatal('Unsupported PRJ_PLATFORM: %s. Supported: %s' % (prj_platform, ', '.join(SUPPORTED_PLATFORMS)))
            
        Logs.info('Using command-line parameters: ')
        Logs.info('  PRJ_NAME=%s, PRJ_PLATFORM=%s' % (prj_name, prj_platform))
        is_auto_detected = False  # Flag to indicate explicit parameters were used
    else:
        # Auto-detect parameters if not provided (like Makefile without parameters)
        auto_name, auto_platform, found_config = auto_detect_project_params()
        
        if auto_name is None or auto_platform is None:
            Logs.error('=== Auto-detection Failed ===')
            Logs.error('Could not auto-detect project parameters.')
            Logs.error('Please either:')
            Logs.error('  1. Specify parameters explicitly:')
            Logs.error('     ./waf configure --prj-name=PROJECT --prj-platform=PLATFORM')
            Logs.error('  2. Or create a .config file by running a defconfig first')
            Logs.error('')
            Logs.error('Supported projects: %s' % ', '.join(SUPPORTED_PROJECTS))
            Logs.error('Supported platforms: %s' % ', '.join(SUPPORTED_PLATFORMS))
            ctx.fatal('Auto-detection failed and no parameters provided')
        
        prj_name = auto_name
        prj_platform = auto_platform
        is_auto_detected = True  # Flag to indicate auto-detection was used
        Logs.info('Auto-detection successful:')
        Logs.info('  Auto-detected PRJ_NAME: %s' % prj_name)
        Logs.info('  Auto-detected PRJ_PLATFORM: %s' % prj_platform)
    
    # Store configuration
    ctx.env.PRJ_NAME = prj_name
    ctx.env.PRJ_PLATFORM = prj_platform
    ctx.env.PRJ_CFLAGS = prj_cflags.split() if prj_cflags else []
    ctx.env.PRJ_LDFLAGS = prj_ldflags.split() if prj_ldflags else []
    ctx.env.PRJ_LINKFLAGS = prj_linkflags.split() if prj_linkflags else []
    ctx.env.PRJ_DEFCONFIG = getattr(Options.options, 'prj_defconfig', None)

    # Setup toolchain
    cross_compile = 'arm-rtems5-'
    try:
        cgcc = ctx.find_program(cross_compile + 'gcc')
        if cgcc:
            ctx.env.CC = cgcc
            ctx.env.LINK_CC = cgcc
            ctx.env.LINK_CXX = cross_compile + 'g++'
            ctx.env.CROSS_COMPILE = cross_compile
            ctx.env.AR = cross_compile + 'ar'
            ctx.env.CPP = cross_compile + 'cpp'
            ctx.env.OBJCOPY = cross_compile + 'objcopy'
            ctx.env.OBJDUMP = cross_compile + 'objdump'
            ctx.env.SIZE = cross_compile + 'size'
            ctx.env.STRIP = cross_compile + 'strip'
            ctx.env.NM = cross_compile + 'nm'  # Add nm tool for symbol extraction
            Logs.info('Using cross compiler: %s' % ctx.env.CC)
        else:
            ctx.env.CROSS_COMPILE = ''
            Logs.warn('Cross compiler not found, using native tools for testing')
    except:
        ctx.env.CROSS_COMPILE = ''
        Logs.warn('Cross compiler not found, using native tools for testing')
    
    # Setup BSP and target
    ctx.env.BSP = prj_name
    ctx.env.TARGET_NAME = prj_name
    
    # Check if BSP directory exists
    bsp_dir_path = os.path.join(ctx.path.abspath(), 'bsp', prj_name)
    if not os.path.exists(bsp_dir_path):
        ctx.fatal('BSP directory not found: bsp/%s' % prj_name)
    ctx.env.BSP_DIR = bsp_dir_path
    
    # Handle defconfig
    config_file = os.path.join(ctx.path.abspath(), '.config')
    defconfig_path = None

    # Priority 1: Manual defconfig specified via --prj-defconfig
    if ctx.env.PRJ_DEFCONFIG:
        defconfig_file = ctx.env.PRJ_DEFCONFIG
        if os.path.dirname(defconfig_file):
            defconfig_path = defconfig_file
        else:
            defconfig_path = os.path.join(ctx.path.abspath(), 'configs', defconfig_file)
        
        if not os.path.exists(defconfig_path):
            ctx.fatal('Specified defconfig file not found: %s' % defconfig_path)
        Logs.info('Using manually specified defconfig: %s' % defconfig_path)

    # Priority 2: Not auto-detected and no manual defconfig, try to find matching defconfig
    elif not is_auto_detected:
        defconfig_filename = '%s_%s_defconfig' % (prj_name, prj_platform)
        potential_path = os.path.join(ctx.path.abspath(), 'configs', defconfig_filename)
        if os.path.exists(potential_path):
            defconfig_path = potential_path
            Logs.info('Found and using matching defconfig: %s' % defconfig_path)
        else:
            Logs.info('No matching defconfig found for %s_%s. Using existing .config if available.' % (prj_name, prj_platform))
    
    # Priority 3: Auto-detection mode, rely on existing .config
    else: # is_auto_detected is True
        defconfig_path = None
        Logs.info('Auto-detection mode: Using existing .config file.')
    
    if defconfig_path:
        try:
            with open(defconfig_path, 'r') as src, open(config_file, 'w') as dst:
                dst.write(src.read())
            Logs.info('Using defconfig: %s' % defconfig_path)
        except Exception as e:
            ctx.fatal('Could not copy defconfig (%s): %s' % (defconfig_path, str(e)))
    
    # Generate config header into build directory
    generated_dir = os.path.join(ctx.bldnode.abspath(), 'generated')
    if not os.path.exists(generated_dir):
        os.makedirs(generated_dir)
    
    generate_config_header_simple(ctx, config_file, os.path.join(generated_dir, 'menuconfig.h'))
    
    # Setup include paths
    ctx.env.INCLUDES = [
        os.path.join(ctx.path.abspath(), 'include/osp'),
        os.path.join(ctx.path.abspath(), 'include/common'),
        os.path.join(ctx.path.abspath(), 'include/drivers'),
        os.path.join(ctx.path.abspath(), 'include/bsp/shared'),
        os.path.join(ctx.path.abspath(), 'include/bsp/%s' % prj_name),
        os.path.join(ctx.bldnode.abspath(), 'generated'),
        os.path.join(ctx.path.abspath(), 'bsp/%s/include' % prj_name),
        os.path.join(ctx.path.abspath(), 'bsp/shared/include'),
        os.path.join(ctx.path.abspath(), 'components/cli_core/include'),
        os.path.join(ctx.path.abspath(), 'components/core_json/include'),
        os.path.join(ctx.path.abspath(), 'components/dshell'),
        os.path.join(ctx.path.abspath(), 'components/flash_partition'),
        os.path.join(ctx.path.abspath(), 'components/log/include'),
        os.path.join(ctx.path.abspath(), 'components/misc/include'),
        os.path.join(ctx.path.abspath(), 'components/nmea/include'),
        os.path.join(ctx.path.abspath(), 'components/sbr'),
        os.path.join(ctx.path.abspath(), 'drivers'),
    ]
    
    # Setup CFLAGS - Match Makefile compilation LOGEc
    # In Makefile: if PRJ_CFLAGS is provided, it completely replaces PLATFORM_CFLAGS
    # BUT we must always include BSP flags for RTEMS compilation to work
    
    # First, get BSP flags for RTEMS (these are ALWAYS needed)
    bsp_cflags = []
    if ctx.env.CROSS_COMPILE:
        bsp_pkg_path = '/opt/rtems/5.1/lib/pkgconfig'
        bsp_pkg_lib = 'arm-rtems5-stars'
        try:
            ctx.env.prepend_value('PKG_CONFIG_PATH', bsp_pkg_path)
            ctx.check_cfg(package=bsp_pkg_lib, args='--cflags', uselib_store='BSP')
            if ctx.env.CFLAGS_BSP:
                bsp_cflags = ctx.env.CFLAGS_BSP
                Logs.info('Using pkg-config for BSP flags: %s' % ' '.join(bsp_cflags))
            else:
                raise Exception("pkg-config returned empty flags")
        except Exception as e:
            Logs.warn('Using fallback RTEMS flags')
            # Use Makefile-compatible RTEMS flags instead of WAF defaults
            bsp_cflags = [
                '-qrtems',
                '-march=armv7-r',      # Match Makefile architecture
                '-mapcs',              # Match Makefile APCS
                '-mfloat-abi=soft',    # Match Makefile float ABI
                '-fno-omit-frame-pointer',  # Match Makefile frame pointer
                '-I/opt/rtems/5.1/arm-rtems5/stars/lib/include',
                '-I/opt/rtems/5.1/arm-rtems5/include'
            ]
    else:
        bsp_cflags = []
        Logs.warn('Native compilation - RTEMS headers not available')
    
    if ctx.env.PRJ_CFLAGS and len(ctx.env.PRJ_CFLAGS) > 0:
        # Custom CFLAGS provided - use them as base (like Makefile: PLATFORM_CFLAGS := $(PRJ_CFLAGS))
        # BUT we must always add BSP flags for RTEMS compilation
        Logs.info('Using custom CFLAGS as base ')
        platform_cflags = ctx.env.PRJ_CFLAGS[:] + bsp_cflags  # Custom flags + required BSP flags
    else:
        # No custom CFLAGS - use default platform flags (like Makefile's pkg-config)
        Logs.info('Using default platform CFLAGS ')
        
        # Default platform flags to match Makefile behavior
        default_platform_flags = [
            # Basic warning flags to match Makefile
            '-O2',
            '-g',
            '-Wall',
            '-Werror',
            '-Wunused',
            '-Wno-error=strict-prototypes',
            '-Wmissing-prototypes',
            '-Wimplicit-function-declaration',
            '-Wstrict-prototypes',
            '-Wnested-externs',
            '-ffunction-sections',
            '-fdata-sections'
        ]
        
        platform_cflags = default_platform_flags + bsp_cflags
    
    # Add required common flags (like Makefile: PLATFORM_CFLAGS += -Wno-missing-prototypes ...)
    required_common_flags = [
        '-Wno-missing-prototypes',
        '-Wno-strict-prototypes', 
        '-fstrict-volatile-bitfields'
    ]
    
    # Final CFLAGS assembly (remove duplicates)
    all_cflags = platform_cflags + required_common_flags
    seen = set()
    ctx.env.CFLAGS = []
    for flag in all_cflags:
        if flag not in seen:
            seen.add(flag)
            ctx.env.CFLAGS.append(flag)
    
    # Setup LDFLAGS - Match Makefile linking behavior
    # In Makefile: if PRJ_LDFLAGS is provided, it completely replaces PLATFORM_LDFLAGS
    
    if ctx.env.PRJ_LDFLAGS and len(ctx.env.PRJ_LDFLAGS) > 0:
        # Custom LDFLAGS provided - use them as base (like Makefile: PLATFORM_LDFLAGS := $(PRJ_LDFLAGS))
        Logs.info('Using custom LDFLAGS as base')
        platform_ldflags = ctx.env.PRJ_LDFLAGS[:]  # Copy custom flags
    else:
        # No custom LDFLAGS - use default platform flags (like Makefile's default)
        Logs.info('Using default platform LDFLAGS')
        
        # Default platform LDFLAGS to match Makefile behavior
        platform_ldflags = [
            '-march=armv7-r',
            '-fno-omit-frame-pointer',
            '-mapcs',
            '-mfloat-abi=soft',
            '-ffunction-sections',
            '-fdata-sections',
            '-Wl,--gc-sections',
        ]
    
    # Add extra LDFLAGS based on cross compilation
    if ctx.env.CROSS_COMPILE:
        # Get BSP CFLAGS for LDFLAGS (need the same base flags)
        ldflags_bsp_flags = []
        try:
            if hasattr(ctx.env, 'CFLAGS_BSP') and ctx.env.CFLAGS_BSP:
                # Use the BSP flags from CFLAGS configuration
                ldflags_bsp_flags = [flag for flag in ctx.env.CFLAGS_BSP if not flag.startswith('-I')]
            else:
                # Use fallback flags that match CFLAGS
                ldflags_bsp_flags = [
                    '-qrtems',
                    '-march=armv7-r',
                    '-mapcs',
                    '-mfloat-abi=soft'
                ]
        except:
            ldflags_bsp_flags = [
                '-qrtems',
                '-march=armv7-r',
                '-mapcs',
                '-mfloat-abi=soft'
            ]
        
        ldflags_extra = ['-qnolinkcmds'] + ldflags_bsp_flags
    else:
        ldflags_extra = []
    
    # Combine LDFLAGS
    ctx.env.LDFLAGS = platform_ldflags + ldflags_extra
    
    # Setup static libraries and library paths - exclude rtemsdefaultconfig as project has its own config
    ctx.env.STLIB = ['rtemsbsp', 'c', 'gcc']
    ctx.env.STLIBPATH = ['/opt/rtems/5.1/arm-rtems5/stars/lib', '/opt/rtems/5.1/arm-rtems5/lib']

    # Setup linker script
    ctx.env.LDSCRIPT = 'bsp/%s/linkcmds.bsp' % prj_name
    
    # Print configuration summary
    Logs.info('=== Build Configuration ===')
    Logs.info('PRJ_NAME: %s' % ctx.env.PRJ_NAME)
    Logs.info('PRJ_PLATFORM: %s' % ctx.env.PRJ_PLATFORM)
    Logs.info('PRJ_CFLAGS: %s' % ctx.env.CFLAGS)
    Logs.info('PRJ_LDFLAGS: %s' % ctx.env.LDFLAGS)
    if prj_linkflags:
        Logs.info('PRJ_LINKFLAGS: %s' % prj_linkflags)
    if ctx.env.PRJ_DEFCONFIG:
        Logs.info('PRJ_DEFCONFIG: %s' % ctx.env.PRJ_DEFCONFIG)
    Logs.info('===========================')

def generate_config_header_simple(ctx, config_file, header_file):
    """Generate a simple config header from .config file"""
    header_content = ['#ifndef __CONFIG_H__', '#define __CONFIG_H__', '']

    ctx.env.DEFINES = []

    if os.path.exists(config_file):
        with open(config_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        if value == 'y':
                            header_content.append('#define %s 1' % key)
                            ctx.env.DEFINES.append('%s=1' % key)
                        elif value.startswith('"') and value.endswith('"'):
                            header_content.append('#define %s %s' % (key, value))
                            ctx.env.DEFINES.append('%s=%s' % (key, value))
                        else:
                            header_content.append('#define %s %s' % (key, value))
                            ctx.env.DEFINES.append('%s=%s' % (key, value))
    
    header_content.extend(['', '#endif /* __CONFIG_H__ */'])
    
    with open(header_file, 'w') as f:
        f.write('\n'.join(header_content))
    
    Logs.info('Generated config header: %s' % header_file)

def generate_version_header_simple(bld):
    """Generate auto_version.h"""
    version_dir = os.path.join(bld.bldnode.abspath(), 'generated')
    if not os.path.exists(version_dir):
        os.makedirs(version_dir)
    
    # Get git version - simplified
    git_version = 'unknown'
    try:
        import subprocess
        git_version = subprocess.check_output(['git', 'describe', '--dirty', '--always'], 
                                            cwd=bld.path.abspath()).strip()
        if isinstance(git_version, bytes):
            git_version = git_version.decode('utf-8')
    except:
        pass
    
    # Get build time
    import datetime
    build_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
    
    # Generate header content
    header_content = '''#ifndef __AUTO_VERSION_H__
#define __AUTO_VERSION_H__

#define MINISOC_BSP_NAME "%s"
#define MINISOC_BUILD_TIME "%s"
#define MINISOC_GIT_VERSION "%s"

#endif /* __AUTO_VERSION_H__ */
''' % (bld.env.BSP, build_time, git_version)
    
    version_header = os.path.join(version_dir, 'auto_version.h')
    with open(version_header, 'w') as f:
        f.write(header_content)

def build(bld):
    """Build the project"""
    if not bld.env.PRJ_NAME:
        bld.fatal('Project not configured. Run: waf configure --prj-name=PROJECT --prj-platform=PLATFORM --prj-cflags="FLAGS"')
    
    # Generate version header
    generate_version_header_simple(bld)
    
    # Process linker script
    processed_ld = bld.path.find_or_declare('%s.ld' % bld.env.TARGET_NAME)
    if os.path.exists(bld.env.LDSCRIPT):
        # Construct absolute path to the generated header directory
        generated_header_dir = os.path.join(bld.bldnode.abspath(), 'generated')
        
        bld(
            rule='${CPP} -I%s -x assembler-with-cpp -std=c99 -P -o ${TGT} ${SRC}' % generated_header_dir,
            source=bld.env.LDSCRIPT,
            target=processed_ld,
            name='process_linkerscript'
        )
    
    if not hasattr(bld.env, 'PROJECT_SOURCES'):
        bld.env.PROJECT_SOURCES = []

    bld.recurse('applications')
    bld.recurse('bsp')
    bld.recurse('drivers')
    bld.recurse('components')

    # Convert collected source paths into nodes so waf will create compile tasks
    src_files = []
    all_elf_sources = []  # Separate application files from library files
    
    for src in getattr(bld.env, 'PROJECT_SOURCES', []):
        node = bld.path.find_node(src)
        if node:
            # Separate application files from library files
            if 'applications/' in src:
                all_elf_sources.append(node)
            elif 'drv_symbols.c' in src:
                # Exclude drv_symbols.c from initial library - it will be generated later
                continue
            else:
                src_files.append(node)
        else:
            Logs.warn('Collected source not found: %s' % src)

    # SBR binary embedding support
    if bld.is_defined('CONFIG_SBR_IMG_IN_RAM'):
        # Find sbr.bin in user_config directory
        sbr_bin_node = bld.path.find_node('bsp/' + bld.env.TARGET_NAME + '/sbr.bin')
        if sbr_bin_node:
            # Convert sbr.bin to object file
            # Use the same approach as Makefile: cd to source dir and use relative names
            sbr_obj_target = 'bsp/' + bld.env.TARGET_NAME + '/sbr_embedded.o'
            bld(
                rule='cd ${SRC[0].parent.abspath()} && ${OBJCOPY} -I binary -O elf32-littlearm -B arm --set-section-alignment .data=4 ${SRC[0].name} ${TGT[0].abspath()}',
                source=sbr_bin_node,
                target=sbr_obj_target,
                name='sbr_embed'
            )
            # Add the generated object file to link sources
            sbr_obj_node = bld.path.find_or_declare(sbr_obj_target)
            src_files.append(sbr_obj_node)
        else:
            bld.fatal('sbr.bin not found in bsp/%s directory' % bld.env.TARGET_NAME)

    Logs.info('Library sources:\n%s', '\n'.join([n.abspath() for n in src_files]))
    Logs.info('Application sources:\n%s', '\n'.join([n.abspath() for n in all_elf_sources]))

    # Create lib directory
    lib_dir = os.path.join(bld.out_dir, 'lib')
    if not os.path.exists(lib_dir):
        os.makedirs(lib_dir)
    
    # Step 1: Build static library from all sources (excluding drv_symbols.c)
    bld.stlib(
        features="c cstlib",
        source=src_files,
        target='lib/minisoc',  # WAF will add 'lib' prefix automatically 
        includes=bld.env.INCLUDES,
        name='libminisoc'
    )

    # Step 2: Generate drv_symbols.c from the static library
    symbols_src = bld.path.find_or_declare('components/dshell/drv_symbols.c')
    symbols_sym = bld.path.find_or_declare('components/dshell/drv_symbols.sym')
    symbols_sh = bld.path.find_node('components/dshell/drv_symbols.sh')
    
    # Generate symbol file using nm on the static library
    bld(
        rule='${NM} -A ${SRC} | grep "[BCDT]" | sort -k 3 > ${TGT}',
        source='lib/libminisoc.a',
        target=symbols_sym,
        after=['libminisoc'],  # Ensure this runs after the library is built
        name='generate_symbols'
    )
    
    # Generate drv_symbols.c from symbol file
    bld(
        rule='${SRC[0].abspath()} ${SRC[1].abspath()} > ${TGT}',
        source=[symbols_sh, symbols_sym],
        target=symbols_src,
        after=['generate_symbols'],  # Ensure this runs after symbols are generated
        name='generate_symbols_c'
    )
    
    # Step 3: Compile drv_symbols.c
    bld(
        features='c',
        source=symbols_src,
        # The target is an object file, but we give the task a name to refer to it.
        target='symbols.o', 
        name='symbols_obj', # This name is used by 'use'
        includes=bld.env.INCLUDES,
        after=['generate_symbols_c'],
    )

    target_name = bld.env.TARGET_NAME
    elf_target = target_name + '.elf'
    
    # Link with map file generation
    # Match Makefile linkflags LOGEc - if PRJ_LINKFLAGS provided, use them instead
    if bld.env.PRJ_LINKFLAGS and len(bld.env.PRJ_LINKFLAGS) > 0:
        # Custom LINKFLAGS provided - use them as base (like Makefile: PLATFORM_LINKFLAGS := $(PRJ_LINKFLAGS))
        Logs.info('Using custom LINKFLAGS ')
        linkflags = bld.env.PRJ_LINKFLAGS[:]  # Copy custom flags
    else:
        # No custom LINKFLAGS - use default linkflags (like Makefile default)
        Logs.info('Using default LINKFLAGS ')
        linkflags = [
            '--specs=/opt/rtems/5.1/arm-rtems5/stars/lib/bsp_specs', 
            '-Wl,-T%s' % processed_ld.abspath(), 
            '-Wl,--wrap,__getreent',
            '-Wl,-Map,%s.map' % target_name,  # Generate map file
        ]

    # Use a specific program task to handle the circular dependency with --start-group/--end-group
    prog = bld.program(
        features="c cprogram",
        source=all_elf_sources,  # Application + RTEMS config object files
        target=elf_target,
        includes=bld.env.INCLUDES,
        use=['libminisoc', 'symbols_obj'],  # Use our static library and the symbols object
        stlib=getattr(bld.env, 'STLIB', []),
        stlibpath=getattr(bld.env, 'STLIBPATH', []),
        linkflags=linkflags,
    )
    
    # Manually wrap the used libraries with --start-group and --end-group
    # by modifying the task generator's environment. This is the correct Waf way.
    prog.env.prepend_value('STLIB_MARKER', '-Wl,--start-group')
    prog.env.append_value('STLIB_MARKER', '-Wl,--end-group')

    # Step 5: Generate bin and hex files from ELF
    elf_path = os.path.join(bld.out_dir, elf_target)
    bin_target = target_name + '.bin'
    hex_target = target_name + '.hex'
    
    # Generate binary file using objcopy
    bld(
        rule='${OBJCOPY} -O binary ${SRC} ${TGT}',
        source=elf_target,
        target=bin_target,
        after=[elf_target],
        name='generate_bin'
    )
    
    # Generate hex file using od
    bld(
        rule='od -v -w32 -t x4 -A n ${SRC} > ${TGT}',
        source=bin_target,
        target=hex_target,
        after=['generate_bin'],
        name='generate_hex'
    )
    
    # Add a post-build task to generate library artifacts
    bld.add_post_fun(lambda ctx: generate_library_artifacts_post(ctx, processed_ld))

def generate_library_artifacts_post(ctx, processed_ld):
    """Post-build library artifacts generation"""
    import shutil
    
    # Setup paths
    lib_dir = os.path.join(ctx.bldnode.abspath(), 'lib')
    dest_include_dir = os.path.join(lib_dir, 'include')
    src_include_dir = os.path.join(ctx.path.abspath(), 'include')

    # Ensure lib directory exists
    if not os.path.exists(lib_dir):
        os.makedirs(lib_dir)

    # Copy the entire top-level include directory to build/lib/include
    try:
        if os.path.exists(dest_include_dir):
            shutil.rmtree(dest_include_dir)
        shutil.copytree(src_include_dir, dest_include_dir)
        os.system('chmod -R 777 %s' % dest_include_dir)
        Logs.info('Copied %s to %s' % (src_include_dir, dest_include_dir))
    except Exception as e:
        Logs.error('Failed to copy include directory: %s' % str(e))

    # Copy linker script
    if processed_ld:
        linkcmds_path = os.path.join(lib_dir, 'linkcmds.bsp')
        shutil.copy2(processed_ld.abspath(), linkcmds_path)
    
    # Copy symbols generation script
    symbols_script_src = os.path.join(ctx.path.abspath(), 'components', 'dshell', 'drv_symbols.sh')
    if os.path.exists(symbols_script_src):
        symbols_script_dst = os.path.join(lib_dir, 'drv_symbols.sh')
        shutil.copy2(symbols_script_src, symbols_script_dst)
        os.chmod(symbols_script_dst, 0o755)  # Make executable
    
def clean(ctx):
    """Clean build artifacts"""
    import shutil
    
    # Remove build directory
    if os.path.exists(out):
        shutil.rmtree(out)
        Logs.info('Removed build directory')
    
# Define custom command classes
class MenuconfigContext(Context.Context):
    cmd = 'menuconfig'
    
    def execute(self):
        """Run menuconfig"""
        Logs.info('Menuconfig not implemented in simplified version')
        Logs.info('Please edit .config file manually or use the original Make system')

class CleanContext(Context.Context):
    cmd = 'clean'
    fun = 'clean'
