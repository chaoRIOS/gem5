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

#include "cpu/vector_engine/vmu/vector_mem_unit.hh"

#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "base/logging.hh"
#include "debug/VectorMemUnit.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"
namespace gem5
{

namespace RiscvISA
{

VectorMemUnit::VectorMemUnit(const VectorMemUnitParams &params) :
    SimObject(SimObjectParams(params)), occupied(false),
    memReader(params.memReader), memReader_addr(params.memReader_addr),
    memWriter(params.memWriter)
{}

VectorMemUnit::~VectorMemUnit() {}

bool
VectorMemUnit::isOccupied()
{
    return occupied;
}

void
VectorMemUnit::issue(VectorEngine &vector_wrapper,
        RiscvISA::VectorStaticInst &insn, VectorDynInst *dyn_insn,
        ExecContextPtr &xc, uint64_t src1, uint64_t src2, uint64_t vtype,
        uint64_t vl, std::function<void(Fault fault)> done_callback)
{
    /* Note:
     * ========================================================
     *  Vector Load/Store Addressing Modes   - version 1.0
     * ========================================================
     *  mop [1:0] encoding for loads
     *     0 0  unit-stride                 VLE
     *     0 1  indexed-unordered           VLUXEI
     *     1 0  strided                     VLSE
     *     1 1  indexed-ordered             VLOXEI
     *
     *  mop [2:0] encoding for stores
     *     0 0  unit-stride                 VSE
     *     0 1  indexed-unordered           VSUXEI
     *     1 0  strided                     VSSE
     *     1 1  indexed-ordered             VSOXEI
     *
     */

    // If vl == 0 then callback, means that the VL = 0
    // @TODO: add vstart & vl tailing policy checking
    if (vl == 0) {
        occupied = false;
        done_callback(NoFault);
        return;
    }
    assert(!occupied);
    occupied = true;

    vectorwrapper = &vector_wrapper;

    // Get vector config

    // sew in bit
    uint64_t sew = vectorwrapper->vector_config->get_vtype_sew(vtype);
    float lmul = vectorwrapper->vector_config->get_vtype_lmul(vtype);

    // eew and emul
    uint64_t eew = 0;
    float emul = 0;

    // VLEN
    uint64_t vlen = vectorwrapper->vector_config->get_mvl_lmul1_bits();
    // max vl in bit
    // max vl = VLEN * emul
    uint64_t mvl_bits =
            vectorwrapper->vector_config->get_max_vector_length_bits(vtype);
    // max vl in elem number
    // max vl elem = VLEN * emul / sew
    uint64_t mvl_elem =
            vectorwrapper->vector_config->get_max_vector_length_elem(vtype);

    if (mvl_elem < vl) {
        panic("Vector register group %d * %d = %d bits is insufficient "
              "for vl(%d) * sew(%d)",
                emul, vlen, mvl_bits, vl, sew);
    }

    // Get instruction infomation

    // mop
    uint8_t mop = insn.mop();
    bool unit_strided = (mop == 0);
    bool indexed_unordered = (mop == 1);
    bool strided = (mop == 2);
    bool indexed_ordered = (mop == 3);
    bool indexed = indexed_unordered || indexed_ordered;

    // extended mop
    uint8_t lumop = insn.lumop();
    uint8_t sumop = insn.sumop();
    bool whole_register = (mop == 0) && ((lumop == 0x8) || (sumop == 0x8));

    // elem_width in byte
    uint8_t elem_width = 0;
    // index width in byte
    uint8_t index_width = 0;
    if (indexed) {
        switch (insn.func3()) {
        case 0:
            index_width = 1;
            break;
        case 5:
            index_width = 2;
            break;
        case 6:
            index_width = 4;
            break;
        case 7:
            index_width = 8;
            break;
        default:
            index_width = 0;
            break;
        }
        eew = sew / lmul;
    } else {
        switch (insn.func3()) {
        case 0:
            eew = 8;
            break;
        case 5:
            eew = 16;
            break;
        case 6:
            eew = 32;
            break;
        case 7:
            eew = 64;
            break;
        default:
            eew = 0;
            break;
        }
    }

    elem_width = eew / 8;
    assert(elem_width != 0);

    emul = lmul * (float)eew / (float)sew;

    assert(emul <= 8);
    assert(emul >= 1 / 8);

    // stride in byte
    uint64_t stride = (strided) ? src2 : elem_width;

    // nfields
    uint8_t nf = insn.nf();
    uint8_t nfields = nf + 1;

    bool segment = (nf) && (!lumop || !sumop);

    // The product EMUL * NFIELDS represents the number of underlying vector
    // registers that will be touched by a segmented load or store
    // instruction. This constraint makes this total no larger than 1/4 of the
    // architectural register file, and the same as for regular operations with
    // EMUL=8
    assert(emul * nfields <= 8);

    // whole_register instructions
    uint64_t evl = 0;
    if (whole_register) {
        evl = nfields * vlen / eew;
    } else {
        evl = vl;
    }

    // logging
    std::stringstream mem_mop;
    if (indexed_ordered) {
        mem_mop << "indexed_ordered"
                << " ei" << index_width;
    } else if (indexed_unordered) {
        mem_mop << "indexed_unordered"
                << " ei" << index_width;
    } else if (unit_strided) {
        mem_mop << "unit_strided";
    } else if (strided) {
        mem_mop << "strided (" << stride << ")";
    } else {
        mem_mop << " ";
    }

    // Organize memory access request
    bool is_load = insn.isLoad();
    bool is_store = insn.isStore();
    assert(is_load || is_store);

    uint64_t mem_addr_dest =
            is_load ? (uint64_t)dyn_insn->get_renamed_dst() * mvl_bits / 8 :
                      src1;
    uint64_t mem_addr_data =
            is_load ? src1 :
                      (uint64_t)dyn_insn->get_renamed_src3() * mvl_bits / 8;
    uint64_t mem_addr_index =
            indexed ? (uint64_t)dyn_insn->get_renamed_src2() * mvl_bits / 8 :
                      0;

    memWriter->initialize(
            vector_wrapper, evl, elem_width, index_width, mem_addr_dest,
            is_load ? 0 : mop, stride, nfields, emul, is_load ? 1 : 0, xc,
            [done_callback, dyn_insn, this](bool done) {
                if (done) {
                    this->occupied = false;
                    done_callback(NoFault);
                }
            },
            whole_register, segment);
    if (indexed) {
        memReader_addr->initialize(vector_wrapper, evl, index_width, 0,
                mem_addr_index, 0, stride, nfields, emul, 1, xc,
                [is_load, index_width, this](
                        uint8_t *data, uint8_t size, bool done) {
                    uint8_t *ndata = new uint8_t[index_width];
                    memcpy(ndata, data, index_width);
                    DPRINTF(VectorMemUnit, "queue Data index addr 0x%x \n",
                            *(uint64_t *)ndata & mask(index_width * 8));
                    if (is_load) {
                        this->memReader->queueData(ndata);
                    } else {
                        this->memWriter->queueAddrs(ndata);
                    }
                    delete[] data;
                });
    }

    memReader->initialize(
            vector_wrapper, evl, elem_width, index_width, mem_addr_data,
            is_load ? mop : 0, stride, nfields, emul, is_load ? 0 : 1, xc,
            [is_load, mvl_elem, evl, elem_width, this](
                    uint8_t *data, uint8_t size, bool done) {
                uint8_t *ndata = new uint8_t[elem_width];
                memcpy(ndata, data, elem_width);
                DPRINTF(VectorMemUnit, "queue Data 0x%x \n",
                        *(uint64_t *)ndata & mask(elem_width * 8));
                this->memWriter->queueData(ndata);
                    delete[] data;
                    // @TODO: Tailing policy
            },
            whole_register, segment);
}

} // namespace RiscvISA

} // namespace gem5
