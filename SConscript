# RT-Thread building script for bridge

from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add bmi088 src files.
src += Glob('src/sensor_intf_bmi088.c')
src += Glob('src/bmi088.c')

# add bmi088 include path.
path  = [cwd, cwd + '/inc']

# add src and include to group.
group = DefineGroup('bmi088', src, depend = ['PKG_USING_BMI088'], CPPPATH = path)

Return('group')
