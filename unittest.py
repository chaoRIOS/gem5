import os
import pathlib
import subprocess

test_path = os.path.join(str(pathlib.Path.home()), 'work', 'rvv-test64') 

tests = os.listdir(test_path)
tests = [test for test in tests if '-v-' not in test and '-vf' not in test and '-vmf' not in test]
done_item = []
fail_item = []
for test in tests:
    subp = subprocess.run(['build/RISCV/gem5.opt', 'configs/example/riscv_vector_engine.py', '--cmd=' + os.path.join(test_path, test)], stderr=2)
    if subp.returncode == 0:
        done_item.append(test)
    else:
        fail_item.append(test)
    # break
print(fail_item)
print(len(done_item), "of", len(tests), "tests passed")

# print(tests)
