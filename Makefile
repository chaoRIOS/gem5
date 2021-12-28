GEM5_PATH=build/RISCV/gem5.opt
CONFIG_PATH=configs/example/riscv_vector_engine.py

WORKLOAD_PATH=../rvv-intrinsic-doc/examples/profile.out

run:
	$(GEM5_PATH) $(CONFIG_PATH) --cmd=$(WORKLOAD_PATH)

rebuild:
	scons $(GEM5_PATH) -j$$(nproc)