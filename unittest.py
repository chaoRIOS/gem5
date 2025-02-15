import os
import pathlib
import subprocess

test_path = os.path.join(str(pathlib.Path.home()), 'work', 'rvv-test64') 

tests = os.listdir(test_path)
tests = [test for test in tests if '-v-' not in test and '-vf' not in test and '-vmf' not in test]


# Fail tests: 65
tests = [

    # load/store

    # 'rv64uv-p-vle',
    # 'rv64uv-p-vle8ff',
    # 'rv64uv-p-vle16ff',
    # 'rv64uv-p-vle32ff',
    # 'rv64uv-p-vle64ff',
    # 'rv64uv-p-vlre',
    # 'rv64uv-p-vlse',
    # 'rv64uv-p-vlseg',
    # 'rv64uv-p-vlsseg',
    # 'rv64uv-p-vlxseg',
    # 'rv64uv-p-vlxei',

    # 'rv64uv-p-vse',
    # 'rv64uv-p-vsre',
    # 'rv64uv-p-vsse',
    # 'rv64uv-p-vsseg',
    # 'rv64uv-p-vssseg',
    # 'rv64uv-p-vsxseg',
    # 'rv64uv-p-vsxei',

    # 15 mask
    # 'rv64uv-p-viota', # chap 15, mask instruction
    # 'rv64uv-p-vsfmb',

    # 16 permutation
    # 'rv64uv-p-vcompress', # datapath
    # 'rv64uv-p-vmre', # vmvnr, vmvnr is wrongly ahead of vle
    # 'rv64uv-p-vrgather',

    # 12
    # 'rv64uv-p-vnclip',# test_2 ?
    # 'rv64uv-p-vnclipu',

    # Seg
    # 'rv64uv-p-vlseg',
    # 'rv64uv-p-vlsseg',
    # 'rv64uv-p-vlxseg',
    # 'rv64uv-p-vsseg',
    # 'rv64uv-p-vssseg',
    # 'rv64uv-p-vsxseg', # vsuxseg

    # Whole register load/store: vl<nf>r
    # 'rv64uv-p-vlre', 
    # 'rv64uv-p-vsre',

    # 11 widening
    # 'rv64uv-p-vwredsum',
    # 'rv64uv-p-vwredsumu',
]
# Done tests: 25
# tests = ['rv64uv-p-vmin', 'rv64uv-p-vmaxu', 'rv64uv-p-vredmax', 'rv64uv-p-vand', 'rv64uv-p-vredmin', 'rv64uv-p-vredand', 'rv64uv-p-vxor', 'rv64uv-p-vle8ff', 'rv64uv-p-vredsum', 'rv64uv-p-vmul', 'rv64uv-p-vsll', 'rv64uv-p-vlxei', 'rv64uv-p-vredor', 'rv64uv-p-vadd', 'rv64uv-p-vle32ff', 'rv64uv-p-vle16ff', 'rv64uv-p-vredxor', 'rv64uv-p-vle64ff', 'rv64uv-p-vredminu', 'rv64uv-p-vmax', 'rv64uv-p-vor', 'rv64uv-p-vminu', 'rv64uv-p-vredmaxu', 'rv64uv-p-vle', 'rv64uv-p-vsub']
# Sys error tests: 5
# tests = ['rv64uv-p-vmv', 'rv64uv-p-vdiv', 'rv64uv-p-vslide', 'rv64uv-p-vext', 'rv64uv-p-vrem']

done_item = []
fail_item = []
err_item = []
for test in tests[:]:
    subp = subprocess.run(['build/RISCV/gem5.opt', '--debug-flag=MemUnitReadTiming,VectorRegister,VectorMemUnit,VectorLane,Exec,VectorInst,VectorEngine,Registers,Datapath', 'configs/example/riscv_vector_engine.py', '--cmd=' + os.path.join(test_path, test)], stderr=2)
    # subp = subprocess.run(['build/RISCV/gem5.opt', 'configs/example/riscv_vector_engine.py', '--cmd=' + os.path.join(test_path, test)], stderr=2)
    if subp.returncode == 0:
        # result = input(len(done_item))
        result = ''
        if result == 'p':
            done_item.append(test)
        else:
            fail_item.append(test)
    else:
        print(subp.returncode)
        err_item.append(test)
    break
print('done:', len(done_item), done_item)
print('fail:', len(fail_item), fail_item)
print('err: ', len(err_item), err_item)
print(len(done_item), "of", len(tests), "tests passed")
