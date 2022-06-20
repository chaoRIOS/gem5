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

#include "cpu/vector_engine/vpu/vector_config/vector_config.hh"

#include <bitset>
#include <cstdint>
#include <deque>
#include <functional>

#include "debug/VectorConfig.hh"
#include "params/VectorConfig.hh"
#include "sim/faults.hh"
#include "sim/sim_object.hh"

using std::max;
using std::min;

namespace gem5
{

namespace RiscvISA
{
/**
 * VPU local Configuration
 */

VectorConfig::VectorConfig(const VectorConfigParams& params) :
    SimObject(SimObjectParams(params)), max_vector_length(params.max_vl)
{}

VectorConfig::~VectorConfig() {}

uint64_t
VectorConfig::reqAppVectorLength(
        uint64_t vl, uint64_t vl_old, uint64_t rs1, uint64_t rd)
{
    if (rs1 != 0) {
        return vl;
    } else if (rd != 0) {
        return UINT32_MAX;
    } else {
        return vl_old;
    }
}

void
VectorConfig::handleVectorConfig(RiscvISA::VectorStaticInst* inst,
        ExecContextPtr& xc, uint64_t& _vl, uint64_t& _vtype)
{
    if (xc->readMiscReg(RiscvISA::MISCREG_VLENB) != get_vlenb()) {
        xc->setMiscReg(MISCREG_VLENB, get_vlenb());
    }

    bool vsetvl = (inst->func6() == 0x20);
    bool vsetvli = (inst->func6() == 0x00);
    bool vsetivli = (inst->func6() == 0x30);

    uint64_t vl_old = xc->readMiscReg(RiscvISA::MISCREG_VL);
    uint64_t vtype_old = xc->readMiscReg(RiscvISA::MISCREG_VTYPE);

    uint64_t vl = 0;
    uint64_t vtype = 0;

    uint64_t sew = 0;
    float lmul = 0;
    uint64_t vlmax = 0;

    uint64_t avl = 0;

    uint64_t rs1 = inst->vs1();
    uint64_t rd = inst->vd();

    if (vsetvl) {
        vl = xc->readIntRegOperand(inst, 0);
        vtype = xc->readIntRegOperand(inst, 1);

        sew = get_vtype_sew(vtype);
        lmul = get_vtype_lmul(vtype);
        vlmax = lmul * max_vector_length / sew;

        avl = reqAppVectorLength(vl, vl_old, rs1, rd);

        if (rs1 != 0) {
            if (avl <= vlmax) {
                vl = avl;
            } else if (avl < 2 * vlmax) {
                vl = avl;
                vl = min(
                        max(vl, (uint64_t)ceil((float)avl / (float)2)), vlmax);
            } else if (avl >= 2 * vlmax) {
                vl = vlmax;
            }

            _vl = vl;
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VL, vl);
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        } else if (rd != 0) {
            vl = vlmax;
            _vl = vl;
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VL, vl);
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        } else {
            vl = vl_old;
            _vl = vl;

            if (((float)sew / (float)lmul) !=
                    ((float)get_vtype_sew(vtype_old) /
                            (float)get_vtype_lmul(vtype_old))) {
                // set vill
                vtype &= ~(UINT32_MAX >> 1);
            }

            // Not setting vl csr
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        }

    } else if (vsetvli) {
        vl = xc->readIntRegOperand(inst, 0);
        vtype = (uint64_t)(inst->vtype11());

        sew = get_vtype_sew(vtype);
        lmul = get_vtype_lmul(vtype);
        vlmax = lmul * max_vector_length / sew;

        avl = reqAppVectorLength(vl, vl_old, rs1, rd);

        if (rs1 != 0) {
            if (avl <= vlmax) {
                vl = avl;
            } else if (avl < 2 * vlmax) {
                vl = avl;
                vl = min(
                        max(vl, (uint64_t)ceil((float)avl / (float)2)), vlmax);
            } else if (avl >= 2 * vlmax) {
                vl = vlmax;
            }

            _vl = vl;
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VL, vl);
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        } else if (rd != 0) {
            vl = vlmax;
            _vl = vl;
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VL, vl);
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        } else {
            vl = vl_old;
            _vl = vl;

            if (((float)sew / (float)lmul) !=
                    ((float)get_vtype_sew(vtype_old) /
                            (float)get_vtype_lmul(vtype_old))) {
                // set vill
                vtype &= ~(UINT32_MAX >> 1);
            }

            // Not setting vl csr
            _vtype = vtype;
            xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);
        }

    } else if (vsetivli) {
        vl = (uint64_t)(inst->imm5());
        vtype = (uint64_t)(inst->vtype10());

        sew = get_vtype_sew(vtype);
        lmul = get_vtype_lmul(vtype);
        vlmax = lmul * max_vector_length / sew;

        avl = vl;

        _vl = vl;
        _vtype = vtype;
        xc->setMiscReg(RiscvISA::MISCREG_VL, vl);
        xc->setMiscReg(RiscvISA::MISCREG_VTYPE, vtype);

    } else {
        panic("Illegal vector config instruction");
    }

    if (inst->vd() != 0) {
        DPRINTF(VectorConfig,
                "Setting register: %d ,"
                " with value : %d\n",
                inst->vd(), vl);
        xc->setIntRegOperand(inst, 0, vl);
    }
}

uint64_t
VectorConfig::vector_length_in_bits(uint64_t vl, uint64_t vtype)
{
    uint64_t vl_bits = 0;
    uint32_t sew = get_vtype_sew(vtype);
    vl_bits = vl * sew;
    return vl_bits;
}

uint64_t
VectorConfig::get_max_vector_length_elem(uint64_t vtype)
{
    uint32_t mvl_elem = 0;
    uint32_t sew = get_vtype_sew(vtype);
    float lmul = get_vtype_lmul(vtype);
    mvl_elem = lmul * max_vector_length / sew;
    return mvl_elem;
}

uint64_t
VectorConfig::get_max_vector_length_bits(uint64_t vtype)
{
    uint32_t mvl_bits = 0;
    // uint32_t sew  =  get_vtype_sew(vtype);
    float lmul = get_vtype_lmul(vtype);
    mvl_bits = lmul * max_vector_length;
    return mvl_bits;
}

uint64_t
VectorConfig::get_mvl_lmul1_bits()
{
    uint32_t mvl_bits = 0;
    mvl_bits = max_vector_length;
    return mvl_bits;
}

float
VectorConfig::get_vtype_lmul(uint64_t vtype)
{
    uint8_t vlmul = vt(vtype, 0, 2);
    float LMUL;

    switch (vlmul) {
    case 0:
        LMUL = 1.0;
        break;
    case 1:
        LMUL = 2.0;
        break;
    case 2:
        LMUL = 4.0;
        break;
    case 3:
        LMUL = 8.0;
        break;
    // case 4:
    // reserved
    case 5:
        LMUL = 1 / 8;
        break;
    case 6:
        LMUL = 1 / 4;
        break;
    case 7:
        LMUL = 1 / 2;
        break;
    default:
        panic("LMUL not implemented\n");
        LMUL = 0;
    }
    return LMUL;
}

uint64_t
VectorConfig::get_vtype_sew(uint64_t vtype)
{
    uint8_t vsew = vt(vtype, 3, 3);
    uint64_t SEW;
    switch (vsew) {
    case 0:
        SEW = 8;
        break;
    case 1:
        SEW = 16;
        break;
    case 2:
        SEW = 32;
        break;
    case 3:
        SEW = 64;
        break;
    default:
        panic("SEW not supported vtype: %d vsew: %d \n", vtype, vsew);
        SEW = 0;
    }
    return SEW;
}

uint64_t
VectorConfig::get_vtype_ediv(uint64_t vtype)
{
    uint8_t vediv = vt(vtype, 5, 2);
    uint64_t EDIV;

    switch (vediv) {
    case 0:
        EDIV = 1;
        break;
    case 1:
        EDIV = 2;
        break;
    case 2:
        EDIV = 4;
        break;
    case 3:
        EDIV = 8;
        break;
    default:
        panic("EDIV not implemented\n");
        EDIV = 0;
    }
    return EDIV;
}

} // namespace RiscvISA

} // namespace gem5
