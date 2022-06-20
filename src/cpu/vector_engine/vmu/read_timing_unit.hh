/*
 * Copyright (c) 2020 Barcelona Supercomputing Center
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Cristóbal Ramírez
 */

#ifndef __CPU_MEM_UNIT_READ_TIMING__
#define __CPU_MEM_UNIT_READ_TIMING__

#include <cstdint>
#include <functional>
#include <queue>

#include "base/statistics.hh"
#include "cpu/vector_engine/vector_engine.hh"
#include "params/MemUnitReadTiming.hh"
#include "sim/ticked_object.hh"
namespace gem5
{

namespace RiscvISA
{

class VectorEngine;

class MemUnitReadTiming : public TickedObject
{
  public:
    MemUnitReadTiming(const MemUnitReadTimingParams &params);
    ~MemUnitReadTiming();

    // overrides
    void evaluate() override;
    void regStats() override;

    // Note: For indexed instructions, addrs are queued in dataQ
    void queueData(uint8_t *data);

    void initialize(VectorEngine &vector_wrapper, uint64_t vl,
            uint64_t elem_width, uint8_t index_width, uint64_t mem_addr,
            uint8_t mop, uint64_t stride, uint8_t nfields, float emul,
            bool location, ExecContextPtr &xc,
            std::function<void(uint8_t *, uint8_t, bool)> on_item_load);

  private:
    // set by params
    const uint8_t channel;
    const uint64_t cacheLineSize;
    const uint64_t VRF_LineSize;
    const uint64_t VLEN;

    volatile bool done;
    std::deque<uint8_t *> dataQ;

    std::function<bool(void)> readFunction;

    uint64_t vecIndex;
    uint64_t vecFieldIndex;

    VectorEngine *vectorwrapper;

  public:
    // Stat for number of cache lines read requested
    Stats::Scalar Cache_line_r_req;
};

} // namespace RiscvISA

} // namespace gem5
#endif //__CPU_MEM_UNIT_READ_TIMING__
