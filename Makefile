GEM5_DEBUG_PATH=build/RISCV/gem5.debug
GEM5_OPT_PATH=build/RISCV/gem5.opt
GEM5_PERF_PATH=build/RISCV/gem5.opt --pprof

GEM5_BUILD_PATH=$(GEM5_DEBUG_PATH)
GEM5_RUN_PATH=$(GEM5_DEBUG_PATH)

CONFIG_PATH=configs/example/riscv_vector_engine.py
CPU_PROFILE_PATH=perf~
CPU_PROFILE_DUMP_PATH=perf.pdf~

# WORKLOAD_PATH=$$HOME/work/talon-rvv/my_rvv.elf
WORKLOAD_PATH=/opt/cputest/rvv-test64/rv64uv-p-vadd
# WORKLOAD_PATH=$$HOME/work/rvv-intrinsic-doc/examples/profile.out
workload=$(WORKLOAD_PATH)
run:
	$(GEM5_RUN_PATH) $(CONFIG_PATH) --cmd=$(workload)

perf:
	env CPUPROFILE=$(CPU_PROFILE_PATH) $(GEM5_RUN_PATH) $(CONFIG_PATH) --cmd=$(workload)

perf_dump:
	pprof $(GEM5_RUN_PATH) $(CPU_PROFILE_PATH) --pdf > $(CPU_PROFILE_DUMP_PATH)

scalar_flag=Exec,CpuVectorIssue,MMU,TLB,TLBVerbose,Registers,RiscvMisc,MinorScoreboard
vector_flag=VectorEngineInfo,VectorEngine,VectorLane,VectorInst,VectorEngineInterface,VectorRename,VectorMemUnit,InstQueue,Datapath,MemUnitReadTiming

flag=$(scalar_flag),$(vector_flag)
debug:
	$(GEM5_RUN_PATH) --debug-flag=$(flag) $(CONFIG_PATH) --cmd=$(workload)

rebuild:
	# scons PYTHON_CONFIG=python3-config $(GEM5_BUILD_PATH) -j1
	scons PYTHON_CONFIG=python3-config $(GEM5_BUILD_PATH) -j$$(nproc)

clean:
	rm -rf build/
	
test:
	python unittest.py
