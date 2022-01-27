import os
import pathlib
import subprocess

test_path = os.path.join(str(pathlib.Path.home()), 'work', 'rvv-test64') 

# tests = os.listdir(test_path)
tests=['rv64uv-p-vsra','rv64uv-p-vmflt','rv64uv-p-vmv','rv64uv-p-vsxei','rv64uv-p-vmacc','rv64uv-p-vwaddu','rv64uv-p-vsadd','rv64uv-p-vssra','rv64uv-p-vadc','rv64uv-p-vwredsumu','rv64uv-p-vdiv','rv64uv-p-vremu','rv64uv-p-vwmaccus','rv64uv-p-vasub','rv64uv-v-vadd','rv64uv-p-vlxseg','rv64uv-p-vmulhu_32','rv64uv-p-vsxseg','rv64uv-p-vaadd','rv64uv-p-vssrl','rv64uv-p-vmfgt','rv64uv-p-vnclip','rv64uv-p-vmfeq','rv64uv-p-vlxei','rv64uv-p-vwsub','rv64uv-p-vnmsac','rv64uv-p-vaaddu','rv64uv-p-vasubu','rv64uv-p-vwmul','rv64uv-p-vslide','rv64uv-p-vmulh_32','rv64uv-p-vmulhsu_32','rv64uv-p-vcompress','rv64uv-p-vrsub','rv64uv-p-vwmaccsu','rv64uv-v-vsub','rv64uv-v-vrsub','rv64uv-p-vwmacc','rv64uv-p-vmadd','rv64uv-p-vwmaccu','rv64uv-p-vext','rv64uv-p-vsmul','rv64uv-p-vwredsum','rv64uv-p-vnclipu','rv64uv-p-vmadc_32','rv64uv-p-vrem','rv64uv-p-vdivu_32','rv64uv-p-vssubu','rv64uv-p-vsaddu','rv64uv-p-vwadd','rv64uv-p-vnmsub','rv64uv-p-vwsubu','rv64uv-p-vrgather','rv64uv-p-vssub']
# tests = ['rv64uv-p-vfmax', 'rv64uv-p-vfredmin', 'rv64uv-p-vsra', 'rv64uv-p-vfwredsum', 'rv64uv-p-vmflt', 'rv64uv-p-vmv', 'rv64uv-p-vfsub', 'rv64uv-p-vfwmacc', 'rv64uv-p-vfclass', 'rv64uv-p-vsxei', 'rv64uv-p-vfnmsub', 'rv64uv-p-vfredmax', 'rv64uv-p-vmacc', 'rv64uv-p-vwaddu', 'rv64uv-p-vsadd', 'rv64uv-p-vssra', 'rv64uv-p-vadc', 'rv64uv-p-vwredsumu', 'rv64uv-p-vdiv', 'rv64uv-p-vremu', 'rv64uv-p-vfadd', 'rv64uv-p-vwmaccus', 'rv64uv-p-vasub', 'rv64uv-p-vfrec', 'rv64uv-p-vfsgnj', 'rv64uv-p-vfwcvt', 'rv64uv-v-vadd', 'rv64uv-p-vlxseg', 'rv64uv-p-vmulhu_32', 'rv64uv-p-vsxseg', 'rv64uv-p-vfmerge', 'rv64uv-p-vaadd', 'rv64uv-p-vssrl', 'rv64uv-p-vmfgt', 'rv64uv-p-vfmacc', 'rv64uv-p-vfwnmsac', 'rv64uv-p-vnclip', 'rv64uv-p-vfdiv', 'rv64uv-p-vmfeq', 'rv64uv-p-vlxei', 'rv64uv-p-vwsub', 'rv64uv-p-vnmsac', 'rv64uv-p-vfcvt', 'rv64uv-p-vaaddu', 'rv64uv-p-vasubu', 'rv64uv-p-vfwnmacc', 'rv64uv-p-vfmin', 'rv64uv-p-vfnmacc', 'rv64uv-p-vfredusum', 'rv64uv-p-vwmul', 'rv64uv-p-vslide', 'rv64uv-p-vfmsac', 'rv64uv-p-vmulh_32', 'rv64uv-p-vmulhsu_32', 'rv64uv-p-vcompress', 'rv64uv-p-vrsub', 'rv64uv-p-vwmaccsu', 'rv64uv-v-vsub', 'rv64uv-p-vfwmsac', 'rv64uv-v-vrsub', 'rv64uv-p-vfwmul', 'rv64uv-p-vfredosum', 'rv64uv-p-vwmacc', 'rv64uv-p-vmadd', 'rv64uv-p-vfnmsac', 'rv64uv-p-vwmaccu', 'rv64uv-p-vext', 'rv64uv-p-vsmul', 'rv64uv-p-vfwsub', 'rv64uv-p-vfmadd', 'rv64uv-p-vwredsum', 'rv64uv-p-vfncvt', 'rv64uv-p-vnclipu', 'rv64uv-p-vfmul', 'rv64uv-p-vfnmadd', 'rv64uv-p-vmadc_32', 'rv64uv-p-vrem', 'rv64uv-p-vfwadd', 'rv64uv-p-vfsqrt', 'rv64uv-p-vdivu_32', 'rv64uv-p-vssubu', 'rv64uv-p-vsaddu', 'rv64uv-p-vwadd', 'rv64uv-p-vnmsub', 'rv64uv-p-vwsubu', 'rv64uv-p-vrgather', 'rv64uv-p-vssub', 'rv64uv-p-vfmsub']
done_item = []
fail_item = []
for test in tests:
    subp = subprocess.run(['build/RISCV/gem5.opt', 'configs/example/riscv_vector_engine.py', '--cmd=' + os.path.join(test_path, test)], stderr=2)
    if subp.returncode == 0:
        done_item.append(test)
    else:
        fail_item.append(test)
    break
print(fail_item)
print(len(done_item), "of", len(tests), "tests passed")

