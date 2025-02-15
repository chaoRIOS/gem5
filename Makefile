GEM5_DEBUG_PATH=build/RISCV/gem5.debug
GEM5_OPT_PATH=build/RISCV/gem5.opt
GEM5_PERF_PATH=build/RISCV/gem5.opt --pprof

GEM5_BUILD_PATH=$(GEM5_OPT_PATH)
GEM5_RUN_PATH=$(GEM5_DEBUG_PATH)

CONFIG_PATH=configs/example/riscv_vector_engine.py
CPU_PROFILE_PATH=perf~
CPU_PROFILE_DUMP_PATH=perf.pdf~

# load/store

# 'rv64uv-p-vle', #done
# 'rv64uv-p-vle8ff',
# 'rv64uv-p-vle16ff',
# 'rv64uv-p-vle32ff',
# 'rv64uv-p-vle64ff',
# 'rv64uv-p-vlre', #done
# 'rv64uv-p-vlse', #done
# 'rv64uv-p-vlseg',
# 'rv64uv-p-vlsseg',
# 'rv64uv-p-vlxseg',
# 'rv64uv-p-vlxei', #done

# 'rv64uv-p-vse', #done
# 'rv64uv-p-vsre', #done
# 'rv64uv-p-vsse', #done
# 'rv64uv-p-vsseg',
# 'rv64uv-p-vssseg',
# 'rv64uv-p-vsxseg',
# 'rv64uv-p-vsxei', #done

# WORKLOAD_PATH=/opt/cputest/rvv-test64/
# WORKLOAD_PATH=$(HOME)/work/riscv-tests-vector/isa/
# WORKLOAD_NAME=rv64uv-p-vadd
WORKLOAD_PATH=$(HOME)/work/riscv-vectorized-benchmark-suite/_axpy/bin/
WORKLOAD_NAME=rvv-test
# WORKLOAD_PATH=isa/rv64ui-p-add
# WORKLOAD_PATH=$$HOME/work/talon-rvv/my_rvv.elf
# WORKLOAD_PATH=$$HOME/work/rvv-intrinsic-doc/examples/profile.out
workload=$(WORKLOAD_PATH)$(WORKLOAD_NAME)
run:
	$(GEM5_RUN_PATH) $(CONFIG_PATH) --cmd=$(workload)

perf:
	env CPUPROFILE=$(CPU_PROFILE_PATH) $(GEM5_RUN_PATH) $(CONFIG_PATH) --cmd=$(workload)

perf_dump:
	pprof $(GEM5_RUN_PATH) $(CPU_PROFILE_PATH) --pdf > $(CPU_PROFILE_DUMP_PATH)

scalar_flag=Exec,CpuVectorIssue,Registers#,MinorScoreboard,MinorExecute,Decode,Fetch#,MinorTrace,MinorCPU#,MMU,TLB,TLBVerbose,RiscvMisc
vector_flag=VecRegs,VectorRegister,VectorEngineInfo,VectorEngine,VectorLane,VectorInst,VectorEngineInterface,VectorRename,VectorMemUnit,InstQueue,Datapath,MemUnitReadTiming,MemUnitWriteTiming,InstQueueInst,VectorRegister

flag=$(scalar_flag),$(vector_flag)
debug:
	# gdb \
	$(GEM5_RUN_PATH) \
	--debug-flag=$(flag) $(CONFIG_PATH) --cmd=$(workload)

rebuild:
	# scons PYTHON_CONFIG=python3-config $(GEM5_BUILD_PATH) -j1
	scons PYTHON_CONFIG=python3-config $(GEM5_BUILD_PATH) -j$$(nproc)

clean:
	rm -rf build/
	
test:
	python3 unittest.py
