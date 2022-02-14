import os
import pathlib
import subprocess

test_path = os.path.join(str(pathlib.Path.home()), 'work', 'rvv-test64') 

tests = os.listdir(test_path)
tests = [test for test in tests if '-v-' not in test and '-vf' not in test and '-vmf' not in test]


# Fail tests: 66
tests = [
    'rv64uv-p-vaadd',
    'rv64uv-p-vaaddu',
    'rv64uv-p-vadc',
    'rv64uv-p-vasub',
    'rv64uv-p-vasubu',
    'rv64uv-p-vcompress',
    'rv64uv-p-vdivu_32',
    'rv64uv-p-viota',
    'rv64uv-p-vlre',
    'rv64uv-p-vlse',
    'rv64uv-p-vlseg',
    'rv64uv-p-vlsseg',
    'rv64uv-p-vlxseg',
    'rv64uv-p-vmacc',
    'rv64uv-p-vmadc_32',
    'rv64uv-p-vmadd',
    'rv64uv-p-vmaxu',
    'rv64uv-p-vmre',
    'rv64uv-p-vmrl',
    'rv64uv-p-vmseq',
    'rv64uv-p-vmsgt',
    'rv64uv-p-vmsgtu',
    'rv64uv-p-vmsle',
    'rv64uv-p-vmsleu',
    'rv64uv-p-vmslt',
    'rv64uv-p-vmsltu',
    'rv64uv-p-vmsne',
    'rv64uv-p-vmulh_32',
    'rv64uv-p-vmulhsu_32',
    'rv64uv-p-vmulhu_32',
    'rv64uv-p-vnclip',
    'rv64uv-p-vnclipu',
    'rv64uv-p-vnmsac',
    'rv64uv-p-vnmsub',
    'rv64uv-p-vpopc',
    'rv64uv-p-vremu',
    'rv64uv-p-vrgather',
    'rv64uv-p-vrsub',
    'rv64uv-p-vsadd',
    'rv64uv-p-vsaddu',
    'rv64uv-p-vse',
    'rv64uv-p-vsfmb',
    'rv64uv-p-vsmul',
    'rv64uv-p-vsra',
    'rv64uv-p-vsre',
    'rv64uv-p-vsrl',
    'rv64uv-p-vsse',
    'rv64uv-p-vsseg',
    'rv64uv-p-vssra',
    'rv64uv-p-vssrl',
    'rv64uv-p-vssseg',
    'rv64uv-p-vssub',
    'rv64uv-p-vssubu',
    'rv64uv-p-vsxei',
    'rv64uv-p-vsxseg',
    'rv64uv-p-vwadd',
    'rv64uv-p-vwaddu',
    'rv64uv-p-vwmacc',
    'rv64uv-p-vwmaccsu',
    'rv64uv-p-vwmaccu',
    'rv64uv-p-vwmaccus',
    'rv64uv-p-vwmul',
    'rv64uv-p-vwredsum',
    'rv64uv-p-vwredsumu',
    'rv64uv-p-vwsub',
    'rv64uv-p-vwsubu']
# Done tests: 24
# tests = ['rv64uv-p-vmin', 'rv64uv-p-vredmax', 'rv64uv-p-vand', 'rv64uv-p-vredmin', 'rv64uv-p-vredand', 'rv64uv-p-vxor', 'rv64uv-p-vle8ff', 'rv64uv-p-vredsum', 'rv64uv-p-vmul', 'rv64uv-p-vsll', 'rv64uv-p-vlxei', 'rv64uv-p-vredor', 'rv64uv-p-vadd', 'rv64uv-p-vle32ff', 'rv64uv-p-vle16ff', 'rv64uv-p-vredxor', 'rv64uv-p-vle64ff', 'rv64uv-p-vredminu', 'rv64uv-p-vmax', 'rv64uv-p-vor', 'rv64uv-p-vminu', 'rv64uv-p-vredmaxu', 'rv64uv-p-vle', 'rv64uv-p-vsub']
# Sys error tests: 5
# tests = ['rv64uv-p-vmv', 'rv64uv-p-vdiv', 'rv64uv-p-vslide', 'rv64uv-p-vext', 'rv64uv-p-vrem']

done_item = []
fail_item = []
err_item = []
for test in tests[:]:
    subp = subprocess.run(['build/RISCV/gem5.opt', '--debug-flag=Exec', 'configs/example/riscv_vector_engine.py', '--cmd=' + os.path.join(test_path, test)], stderr=2)
    if subp.returncode == 0:
        result = input(len(done_item))
        if result == 'p':
            done_item.append(test)
        else:
            fail_item.append(test)
    else:
        err_item.append(test)
    # break
print(len(done_item), done_item)
print(len(fail_item), fail_item)
print(len(err_item), err_item)
print(len(done_item), "of", len(tests), "tests passed")
