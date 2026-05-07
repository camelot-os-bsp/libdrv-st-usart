# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: 2026 H2Lab Development Team

project(
  'libdrv-st-usart',
  'c',
  meson_version : '>= 1.3.0',
  version : run_command('support/meson/version.sh', 'get-vcs', check: true).stdout().strip(),
  default_options: [
      'c_std=gnu11',
      'warning_level=3', 'optimization=s',
      'default_library=static', 'b_staticpic=false',
      'licensedir=share/info/libdrv-st-usart',
      'pkgconfig.relocatable=true',
      'config=configs/defconfig',
  ],
)

# import meson needed modules
pkg = import('pkgconfig')

# import Kconfig options
kconfig_file = meson.current_source_dir() / 'Kconfig'
kconfig_proj = subproject('kconfig', default_options: ['kconfig=@0@'.format(kconfig_file)])
kconfig_h = kconfig_proj.get_variable('kconfig_h')
kconfig_data = kconfig_proj.get_variable('kconfig_data')

# drivers need merlin driver framework for dts and io accesses abstractions
merlin_proj = subproject('merlin')
merlin_dep = merlin_proj.get_variable('merlin_dep')

shield_dep = dependency('shield', required : true)

dependencies = []

subdir('include')
subdir('src')

dependencies += [merlin_dep, shield_dep]

# define the library and its public dependency
libdrv_st_usart = static_library(
  'drv-st-usart',
  usart_src,
  dependencies : dependencies,
  include_directories : usart_inc,
  install : true,
)

# allow users to link with the library and include its headers by using the dependency object
libdrv_st_usart_dep = declare_dependency(
  link_with : libdrv_st_usart,
  include_directories : usart_inc,
  dependencies : dependencies,
)
