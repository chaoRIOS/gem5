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

#include "cpu/vector_engine/vmu/write_timing_unit.hh"

#include <algorithm>
#include <bitset>
#include <cassert>
#include <cstring>

#include "base/types.hh"
#include "debug/MemUnitWriteTiming.hh"
namespace gem5
{

namespace RiscvISA
{

MemUnitWriteTiming::MemUnitWriteTiming(
        const MemUnitWriteTimingParams &params) :
    TickedObject(TickedObjectParams(params)),
    channel(params.channel), cacheLineSize(params.cacheLineSize),
    VRF_LineSize(params.VRF_LineSize), VLEN(params.VLEN), done(false)
{}

MemUnitWriteTiming::~MemUnitWriteTiming() {}

void
MemUnitWriteTiming::evaluate()
{
    // Do NOT assert if done here! it coud've finished in < 1 cycle!
    assert(running);
    if (!done) {
        writeFunction();
    }
    if (done) {
        done = false;
        stop();
    }
}

void
MemUnitWriteTiming::regStats()
{
    TickedObject::regStats();

    Cache_line_w_req.name(name() + ".Cache_line_w_req")
            .desc("Number of cache lines write requested");
}

void
MemUnitWriteTiming::queueData(uint8_t *data)
{
    assert(running && !done);
    dataQ.push_back(data);
    DPRINTF(MemUnitWriteTiming, "writer pushing back %#x upto %d\n", *data,
            dataQ.size());
}

void
MemUnitWriteTiming::queueAddrs(uint8_t *data)
{
    assert(running && !done);
    AddrsQ.push_back(data);
}

// to main mem will ALWAYS succeed. vector_reg will be bandwidth throttled
// either way, on writes, we must wait for responses!
//
//  NOTE: delete[]s all data from dataQ if written to fn(), so the
//    exec_context must memcpy the data!
void
MemUnitWriteTiming::initialize(VectorEngine &vector_wrapper, uint64_t vl,
        uint64_t elem_width, uint8_t index_width, uint64_t mem_addr,
        uint8_t mop, uint64_t stride, uint8_t nfields, uint8_t emul,
        bool location, ExecContextPtr &xc,
        std::function<void(bool)> on_item_store)
{
    assert(!running && !done);
    assert(vl > 0);
    assert(!dataQ.size());
    assert(!AddrsQ.size());

    DPRINTF(MemUnitWriteTiming, "writer initialize with %d %d-byte elems\n",
            vl, elem_width);

    vectorwrapper = &vector_wrapper;

    vecIndex = 0;
    vecFieldIndex = 0;

    // This function tries to get up to 'get_up_to' elements from the front of
    // the input queue. if there are less elements, it returns all of them
    // if fn() is successfull, try_write deletes the data therefore, fn()
    // or some downstream function needs to memcpy it!

    // NOTE: be careful with uint16_t overflow here (incase super large
    // vectors)
    auto try_write = [elem_width, this](uint32_t get_up_to,
                             std::function<uint16_t(uint8_t *, uint32_t)> fn) {
        uint64_t can_get = this->dataQ.size();
        if (!can_get) {
            DPRINTF(MemUnitWriteTiming, "try_write dataQ empty\n");
            return false;
        } else if (can_get < get_up_to) {
            DPRINTF(MemUnitWriteTiming, "try_write needs more elements\n");
            return false;
        }
        uint64_t got = get_up_to; // std::min((uint64_t)get_up_to, can_get);
        uint8_t *buf = new uint8_t[got * elem_width];
        for (uint32_t i = 0; i < got; ++i) {
            memcpy(buf + elem_width * i, this->dataQ[i], elem_width);
        }
        uint64_t actually_written;
        if ((actually_written = fn(buf, (uint32_t)got))) {
            delete[] buf;
            for (uint32_t i = 0; i < actually_written; ++i) {
                uint8_t *data = dataQ.front();
                delete[] data;
                dataQ.pop_front();
                DPRINTF(MemUnitWriteTiming, "writer poping downto %d\n",
                        dataQ.size());
            }
            return true;
        } else {
            DPRINTF(MemUnitWriteTiming, "fn(data) failed\n");
            return false;
        }
    };

    // need 'this' for DPRINTF
    auto fin = [on_item_store, vl, this](uint64_t i, uint64_t items_ready) {
        return [on_item_store, vl, i, items_ready, this]() {
            for (uint64_t j = 0; j < items_ready; ++j) {
                bool _done = ((i + j + 1) == vl);
                DPRINTF(MemUnitWriteTiming,
                        "calling on_item_store with"
                        " 'done'=%d\n",
                        _done);
                on_item_store(_done);
            }
        };
    };

    writeFunction = [try_write, location, fin, xc, mem_addr, stride,
                            on_item_store, elem_width, mop, vl,
                            this](void) -> bool {
        // scratch and cache could use different line sizes
        uint64_t line_size;
        switch ((int)location) {
        case 0:
            line_size = cacheLineSize;
            break;
        case 1:
            line_size = VRF_LineSize;
            break;
        default:
            panic("invalid location");
            break;
        }

        // Note: VRF reader is also passed with mop=0
        bool unit_strided = (mop == 0);
        bool indexed_unordered = (mop == 1);
        bool strided = (mop == 2);
        bool indexed_ordered = (mop == 3);
        bool indexed = indexed_unordered || indexed_ordered;

        uint64_t line_addr;
        // uint8_t items_in_line;
        uint64_t addr;
        uint64_t i = this->vecIndex;
        uint64_t consec_items;

        if (unit_strided || strided) // no indexed operation
        {
            // we can always write 1 element
            addr = mem_addr + (strided ? stride : elem_width) * i;
            line_addr = addr - (addr % line_size);
            consec_items = 1;

            DPRINTF(MemUnitWriteTiming,
                    "enqueue data as %d @ %#x, elem_width = %d, line_size = "
                    "%d\n",
                    consec_items, addr, elem_width, line_size);

            // now find any consecutive items that we can also write
            for (uint64_t j = 1;
                    (j < (line_size / elem_width)) && ((i + j) < vl); ++j) {
                uint64_t next_addr =
                        mem_addr + (strided ? stride : elem_width) * (i + j);
                uint64_t next_line_addr = next_addr - (next_addr % line_size);
                if (stride == elem_width && line_addr == next_line_addr) {
                    ++consec_items;
                    DPRINTF(MemUnitWriteTiming,
                            "enqueue data as %d @ %#x, elem_width = %d, "
                            "line_size = %d\n",
                            consec_items, addr, elem_width, line_size);
                } else {
                    break;
                }
            }
        } else if (indexed_ordered) {
            //
        } else if (indexed_unordered) {
            uint64_t can_get = this->AddrsQ.size();
            if (!can_get) {
                DPRINTF(MemUnitWriteTiming, "try_read AddrsQ Addrs empty\n");
                return false;
            }
            uint64_t got = std::min(line_size / elem_width, can_get);
            uint8_t *buf = new uint8_t[got * elem_width];
            for (uint8_t i = 0; i < got; ++i) {
                memcpy(buf + elem_width * i, this->AddrsQ[i], elem_width);
            }

            uint64_t index_addr;
            if (elem_width == 8) {
                index_addr = (uint64_t)((uint64_t *)buf)[0];
            } else if (elem_width == 4) {
                index_addr = (uint64_t)((uint32_t *)buf)[0];
            } else if (elem_width == 2) {
                index_addr = (uint64_t)((uint16_t *)buf)[0];
            } else if (elem_width == 1) {
                index_addr = (uint64_t)((uint8_t *)buf)[0];
            } else {
                panic("invalid mem req elem_width");
            }
            addr = mem_addr + index_addr;
            line_addr = addr - (addr % line_size);
            consec_items = 1;
            delete buf;
        } // end indexed operation

        // now see if there is any data in the queue to write
        DPRINTF(MemUnitWriteTiming, "getting data to write %d items at %#x\n",
                consec_items, addr);

        return try_write(consec_items,
                [fin, location, xc, addr, elem_width, indexed, vl, i, this](
                        uint8_t *data, uint32_t items_ready) -> uint16_t {
                    DPRINTF(MemUnitWriteTiming,
                            "got %d items to write at %#x\n", items_ready,
                            addr);
                    bool success;
                    if (location == 0) { // location = 0 = MEMORY
                        Cache_line_w_req++;
                        success = vectorwrapper->writeVectorMem(addr, data,
                                elem_width * items_ready, xc->tcBase(),
                                this->channel, fin(i, items_ready));
                    } else {
                        uint32_t size_write = elem_width * items_ready;
                        DPRINTF(MemUnitWriteTiming,
                                "size_write %d items to write at"
                                " %#x\n",
                                size_write, addr);
                        success = vectorwrapper->writeVectorReg(addr, data,
                                size_write, this->channel,
                                fin(i, items_ready));
                    }

                    if (success) {
                        if (indexed) {
                            for (uint16_t j = 0; j < items_ready; j++) {
                                this->AddrsQ.pop_front();
                            }
                        }
                        this->vecIndex += items_ready;
                        this->done = (this->vecIndex == vl);
                        return items_ready;
                    }
                    return 0;
                });
    };

    if (writeFunction() && done) {
        done = false;
    } else {
        start();
    }
}

} // namespace RiscvISA

} // namespace gem5