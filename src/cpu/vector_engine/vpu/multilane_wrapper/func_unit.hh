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

#include "arith.h"
#include "debug/Datapath.hh"
namespace gem5
{

namespace RiscvISA
{

float
Datapath::compute_float_fp_op(float Aitem, float Bitem, uint8_t Mitem,
        float Dstitem, RiscvISA::VectorStaticInst* insn)
{
    float Ditem = 0;
    std::string operation = insn->getName();
    numFP32_operations =
            numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfadd_vv") | (operation == "vfadd_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f + %f  = %f  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfsub_vv") | (operation == "vfsub_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f - %f  = %f  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vfmul_vv") | (operation == "vfmul_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f * %f  = %f  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfdiv_vv") || (operation == "vfdiv_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f / %f  = %f\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vfsqrt_v")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? sqrt(Bitem) :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = sqrt (%f)   = %f  \n", Bitem,
                Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point MIN/MAX Instructions
     *************************************************************************/

    if ((operation == "vfmin_vv") || (operation == "vfmin_vf")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %f , %f  = min :%f\n", Aitem,
                Bitem, Ditem);
    }
    if ((operation == "vfmax_vv") || (operation == "vfmax_vf")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %f , %f  = max :%f\n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point Sign-Injection Instructions
     *************************************************************************/

    if ((operation == "vfsgnj_vv") || (operation == "vfsgnj_vf")) {
        Ditem = (Bitem >= 0.0) ? fabs(Aitem) : -Aitem;
        DPRINTF(Datapath, "WB Instruction = %f , %f  = %f\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfsgnjn_vv") || (operation == "vfsgnjn_vf")) {
        Ditem = (Bitem >= 0.0) ? -Aitem : fabs(Aitem);
        DPRINTF(Datapath, "WB Instruction = %f , %f  = %f\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfsgnjx_vv") || (operation == "vfsgnjx_vf")) {
        Ditem = ((Bitem >= 0.0) & (Aitem >= 0.0)) ? Aitem :
                ((Bitem < 0.0) & (Aitem < 0.0))   ? fabs(Aitem) :
                ((Bitem >= 0.0) & (Aitem < 0.0))  ? Aitem :
                                                    -Aitem;
        DPRINTF(Datapath, "WB Instruction = %f , %f  = %f\n", Aitem, Bitem,
                Ditem);
    }

    /**************************************************************************
     *
     *************************************************************************/

    if ((operation == "vfmerge_vf") && (vm == 0)) {
        Ditem = (Mitem == 1) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %f : %f  = %f\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfmv_vf") && (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %f  = %f\n", Aitem, Ditem);
    }

    if ((operation == "vfmacc_vv") || (operation == "vfmacc_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (Aitem * Bitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f * %f + %f  = %f\n", Aitem,
                Bitem, Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv") || (operation == "vfmadd_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (Aitem * Dstitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %f * %f + %f  = %f\n", Aitem,
                Dstitem, Bitem, Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d),"
                " old(%f)  \n",
                Mitem, Dstitem);
    }

    return Ditem;
}

double
Datapath::compute_double_fp_op(double Aitem, double Bitem, uint8_t Mitem,
        double Dstitem, RiscvISA::VectorStaticInst* insn)
{
    float Ditem = 0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfadd_vv") | (operation == "vfadd_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf + %lf  = %lf  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vfsub_vv") | (operation == "vfsub_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf - %lf  = %lf  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vfmul_vv") | (operation == "vfmul_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf * %lf  = %lf  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vfdiv_vv") || (operation == "vfdiv_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf / %lf  = %lf\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vfsqrt_v")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? sqrt(Bitem) :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = sqrt (%lf)   = %lf  \n", Bitem,
                Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point MIN/MAX Instructions
     *************************************************************************/

    if ((operation == "vfmin_vv") || (operation == "vfmin_vf")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %lf , %lf  = min :%lf\n", Aitem,
                Bitem, Ditem);
    }
    if ((operation == "vfmax_vv") || (operation == "vfmax_vf")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %lf , %lf  = max :%lf\n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Floating-Point Sign-Injection Instructions
     *************************************************************************/

    if ((operation == "vfsgnj_vv") || (operation == "vfsgnj_vf")) {
        Ditem = (Bitem >= 0.0) ? fabs(Aitem) : -Aitem;
        DPRINTF(Datapath, "WB Instruction = %lf , %lf  = %lf\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfsgnjn_vv") || (operation == "vfsgnjn_vf")) {
        Ditem = (Bitem >= 0.0) ? -Aitem : fabs(Aitem);
        DPRINTF(Datapath, "WB Instruction = %lf , %lf  = %lf\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfsgnjx_vv") || (operation == "vfsgnjx_vf")) {
        Ditem = ((Bitem >= 0.0) & (Aitem >= 0.0)) ? Aitem :
                ((Bitem < 0.0) & (Aitem < 0.0))   ? fabs(Aitem) :
                ((Bitem >= 0.0) & (Aitem < 0.0))  ? Aitem :
                                                    -Aitem;
        DPRINTF(Datapath, "WB Instruction = %lf , %lf  = %lf\n", Aitem, Bitem,
                Ditem);
    }

    /**************************************************************************
     *
     *************************************************************************/

    if ((operation == "vfmerge_vf") && (vm == 0)) {
        Ditem = (Mitem == 1) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %f : %f  = %f\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vfmv_vf") && (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %f  = %f\n", Aitem, Ditem);
    }

    if ((operation == "vfmacc_vv") || (operation == "vfmacc_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (Aitem * Bitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf * %lf + %lf  = %lf\n", Aitem,
                Bitem, Dstitem, Ditem);
    }

    if ((operation == "vfmadd_vv") || (operation == "vfmadd_vf")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (Aitem * Dstitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %lf * %lf + %lf  = %lf\n", Aitem,
                Dstitem, Bitem, Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d),"
                " old(%f)  \n",
                Mitem, Dstitem);
    }

    return Ditem;
}

double
Datapath::computeDoubleFPReduction(double accumDp, double Bitem, uint8_t Mitem)
{
    double reduction = 0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfredsum_vs") || (operation == "vfredosum_vs")) {
        reduction = (vm == 1) ? accumDp + Bitem :
                    (Mitem)   ? accumDp + Bitem :
                                accumDp;
        DPRINTF(Datapath, " Reduction: Source %lf  Acc= %lf\n", Bitem,
                reduction);
    }

    if (operation == "vfredmax_vs") {
        reduction = (vm == 1) ? ((accumDp > Bitem) ? accumDp : Bitem) :
                    (Mitem)   ? ((accumDp > Bitem) ? accumDp : Bitem) :
                                accumDp;
        DPRINTF(Datapath, " Reduction: Source %lf  Max= %lf\n", Bitem,
                reduction);
    }

    if (operation == "vfredmin_vs") {
        reduction = (vm == 1) ? ((accumDp < Bitem) ? accumDp : Bitem) :
                    (Mitem)   ? ((accumDp < Bitem) ? accumDp : Bitem) :
                                accumDp;
        DPRINTF(Datapath, " Reduction: Source %lf  Min= %lf\n", Bitem,
                reduction);
    }

    return reduction;
}

float
Datapath::computeSingleFPReduction(float accumSp, float Bitem, uint8_t Mitem)
{
    float reduction = 0;
    std::string operation = insn->getName();
    numFP32_operations =
            numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfredsum_vs") || (operation == "vfredosum_vs")) {
        reduction = (vm == 1) ? accumSp + Bitem :
                    (Mitem)   ? accumSp + Bitem :
                                accumSp;
        DPRINTF(Datapath, " Reduction: Source %f  Acc= %f\n", Bitem,
                reduction);
    }

    if (operation == "vfredmax_vs") {
        reduction = (vm == 1) ? ((accumSp > Bitem) ? accumSp : Bitem) :
                    (Mitem)   ? ((accumSp > Bitem) ? accumSp : Bitem) :
                                accumSp;
        DPRINTF(Datapath, " Reduction: Source %f  Max= %f\n", Bitem,
                reduction);
    }

    if (operation == "vfredmin_vs") {
        reduction = (vm == 1) ? ((accumSp < Bitem) ? accumSp : Bitem) :
                    (Mitem)   ? ((accumSp < Bitem) ? accumSp : Bitem) :
                                accumSp;
        DPRINTF(Datapath, " Reduction: Source %f  Min= %f\n", Bitem,
                reduction);
    }

    return reduction;
}

int
Datapath::compute_float_fp_comp_op(
        float Aitem, float Bitem, RiscvISA::VectorStaticInst* insn)
{
    int Ditem = 0;
    std::string operation = insn->getName();
    numFP32_operations =
            numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vmfeq_vv") || (operation == "vmfeq_vf")) {
        Ditem = (Bitem == Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f == %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmfne_vv") || (operation == "vmfne_vf")) {
        Ditem = (Bitem != Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f != %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmflt_vv") || (operation == "vmflt_vf")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f < %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmfle_vv") || (operation == "vmfle_vf")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f <= %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (operation == "vmfgt_vf") {
        Ditem = (Bitem > Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f > %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (operation == "vmfge_vf") {
        Ditem = (Bitem >= Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %f >= %f  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_double_fp_comp_op(
        double Aitem, double Bitem, RiscvISA::VectorStaticInst* insn)
{
    long int Ditem = 0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vmfeq_vv") || (operation == "vmfeq_vf")) {
        Ditem = (Bitem == Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf == %lf  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vmfne_vv") || (operation == "vmfne_vf")) {
        Ditem = (Bitem != Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf != %lf  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vmflt_vv") || (operation == "vmflt_vf")) {
        Ditem = (Bitem < Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf < %lf  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmfle_vv") || (operation == "vmfle_vf")) {
        Ditem = (Bitem <= Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf <= %lf  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if (operation == "vmfgt_vf") {
        Ditem = (Bitem > Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf > %lf  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (operation == "vmfge_vf") {
        Ditem = (Bitem >= Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = %lf >= %lf  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    return Ditem;
}

template <typename T>
bool
get_fixed_rounding_incr(T vec_elem, uint8_t shift_amount, int rounding_mode)
{
    if (shift_amount == 0) {
        return 0;
    } else {
        switch (rounding_mode) {
        case 0b00:
            return slice(vec_elem, shift_amount - 1, 1);
            break;
        case 0b01:
            return (slice(vec_elem, shift_amount - 1, 1) == 0b1) &
                   ((slice(vec_elem, 0, shift_amount - 1) != 0) |
                           (slice(vec_elem, shift_amount, 1) == 0b1));
            break;
        case 0b10:
            return 0;
            break;
        case 0b11:
            return !(slice(vec_elem, shift_amount, 1) == 0b1) &
                   (slice(vec_elem, 0, shift_amount) != 0);
            break;
        default:
            break;
        }
    }
    return 0;
}

template <typename T, typename UT>
T
signed_saturation(int len, T elem, bool& sat)
{
    T elem_sat = 0;
    if ((T)elem > (T)(UT)(mask(len - 1))) {
        // INT_MAX
        elem_sat = mask(len - 1);
        sat = true;
    } else if ((T)elem < (T)(1 << (len - 1))) {
        // INT_MIN
        elem_sat = (T)(1 << (len - 1));
        sat = true;
    } else {
        elem_sat = slice(elem, 0, len);
        sat = false;
    }
    return elem_sat;
}

template <typename T, typename UT>
UT
unsigned_saturation(int len, T elem, bool& sat)
{
    UT elem_sat = 0;
    if (elem > mask(len)) {
        // UINT_MAX
        elem_sat = mask(len);
        sat = true;
    } else {
        elem_sat = slice(elem, 0, len);
        sat = false;
    }
    return elem_sat;
}

long int
Datapath::compute_long_int_op(long int Aitem, long int Bitem, uint8_t Mitem,
        long int Dstitem, RiscvISA::VectorStaticInst* insn)
{
    long int Ditem = 0;
    std::string operation = insn->getName();
    numALU64_operations =
            numALU64_operations.value() + 1; // number of 64-bit ALU operations

    if ((operation == "vsmul_vv") || (operation == "vsmul_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            __int128_t res = (__int128_t)Bitem * (__int128_t)Aitem;
            bool sat = false;
            Ditem = (res >> (64 - 1)) +
                    get_fixed_rounding_incr<__int128_t>(res, 64 - 1,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = signed_saturation<int64_t, uint64_t>(64, Ditem, sat);
            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%d * %d)) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssra_vv") || (operation == "vssra_vx") ||
            (operation == "vssra_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int64_t shift_amount = (Aitem & mask(log2(64UL)));
            Ditem = (Bitem >> shift_amount) +
                    get_fixed_rounding_incr<int64_t>(Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath, "WB Instruction = roundoff_signed(%d, %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssrl_vv") || (operation == "vssrl_vx") ||
            (operation == "vssrl_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int64_t shift_amount = (Aitem & mask(log2(64UL)));
            Ditem = ((uint64_t)Bitem >> shift_amount) +
                    (uint64_t)get_fixed_rounding_incr<uint64_t>(
                            (uint64_t)Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d, %d) = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vaadd_vv") || (operation == "vaadd_vx")) {
        __int128_t res = (__int128_t)Bitem + (__int128_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint64_t)get_fixed_rounding_incr<__int128_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vaaddu_vv") || (operation == "vaaddu_vx")) {
        __uint128_t res =
                (__uint128_t)(uint64_t)Bitem + (__uint128_t)(uint64_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint64_t)get_fixed_rounding_incr<__uint128_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasub_vv") || (operation == "vasub_vx")) {
        __int128_t res = (__int128_t)Bitem - (__int128_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint64_t)get_fixed_rounding_incr<__int128_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasubu_vv") || (operation == "vasubu_vx")) {
        __uint128_t res =
                (__uint128_t)(uint64_t)Bitem - (__uint128_t)(uint64_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint64_t)get_fixed_rounding_incr<__uint128_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vadc_vvm") || (operation == "vadc_vxm") ||
            (operation == "vadc_vim")) {
        Ditem = Aitem + Bitem + Mitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d + %d = %d  \n", Aitem,
                Bitem, Mitem, Ditem);
    }

    if ((operation == "vsbc_vv") || (operation == "vsbc_vx")) {
        Ditem = Bitem - Aitem - Mitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d - %d = %d  \n", Bitem,
                Aitem, Mitem, Ditem);
    }

    if ((operation == "vmadc_vvm") || (operation == "vmadc_vxm") ||
            (operation == "vmadc_vim")) {
        Ditem = (((__uint128_t)(uint64_t)Bitem + (__uint128_t)(uint64_t)Aitem +
                         ((Mitem == 1) ? (__uint128_t)(uint64_t)Mitem :
                                         (__uint128_t)0)) > UINT64_MAX) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d + %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmadc_vv") || (operation == "vmadc_vx") ||
            (operation == "vmadc_vi")) {
        Ditem = ((__uint128_t)(uint64_t)Bitem + (__uint128_t)(uint64_t)Aitem) >
                                UINT64_MAX ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsbc_vvm") || (operation == "vmsbc_vxm")) {
        Ditem = ((__uint128_t)(uint64_t)Bitem <
                        (__uint128_t)(uint64_t)Aitem +
                                (__uint128_t)((Mitem == 1) ? (uint64_t)Mitem :
                                                             0)) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d - %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmsbc_vv") || (operation == "vmsbc_vx")) {
        Ditem = ((uint64_t)Bitem < (uint64_t)Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmacc_vv") || (operation == "vmacc_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Bitem * Aitem + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vnmsac_vv") || (operation == "vnmsac_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Bitem * Aitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vmadd_vv") || (operation == "vmadd_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Dstitem * Aitem + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }
    if ((operation == "vnmsub_vv") || (operation == "vnmsub_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Dstitem * Aitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }

    // ----------------------- @TODO: check implementations above
    // ----------------------

    if ((operation == "vadd_vv") || (operation == "vadd_vx") ||
            (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") ||
            (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrsub_vx") || (operation == "vrsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem - Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsadd_vv") || (operation == "vsadd_vx") ||
            (operation == "vsadd_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_add<int64_t, uint64_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vsaddu_vv") || (operation == "vsaddu_vx") ||
            (operation == "vsaddu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_addu<uint64_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vssub_vv") || (operation == "vssub_vx") ||
            (operation == "vssub_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_sub<int64_t, uint64_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vssubu_vv") || (operation == "vssubu_vx") ||
            (operation == "vssubu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_subu<uint64_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmulh_vv") || (operation == "vmulh_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Aitem < 0) != (Bitem < 0);

            uint64_t Vs1_lo = (uint32_t)abs(Aitem);
            uint64_t Vs1_hi = (uint64_t)abs(Aitem) >> 32;
            uint64_t Vs2_lo = (uint32_t)abs(Bitem);
            uint64_t Vs2_hi = (uint64_t)abs(Bitem) >> 32;

            uint64_t hi = Vs1_hi * Vs2_hi;
            uint64_t mid1 = Vs1_hi * Vs2_lo;
            uint64_t mid2 = Vs1_lo * Vs2_hi;
            uint64_t lo = Vs2_lo * Vs1_lo;
            uint64_t carry = ((uint64_t)(uint32_t)mid1 +
                                     (uint64_t)(uint32_t)mid2 + (lo >> 32)) >>
                             32;

            uint64_t res = hi + (mid1 >> 32) + (mid2 >> 32) + carry;
            Ditem = negate ? ~res + (Aitem * Bitem == 0 ? 1 : 0) : res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhu_vv") || (operation == "vmulhu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            uint64_t Vs1_lo = (uint32_t)(Aitem);
            uint64_t Vs1_hi = (uint64_t)(Aitem) >> 32;
            uint64_t Vs2_lo = (uint32_t)(Bitem);
            uint64_t Vs2_hi = (uint64_t)(Bitem) >> 32;

            uint64_t hi = Vs1_hi * Vs2_hi;
            uint64_t mid1 = Vs1_hi * Vs2_lo;
            uint64_t mid2 = Vs1_lo * Vs2_hi;
            uint64_t lo = Vs2_lo * Vs1_lo;
            uint64_t carry = ((uint64_t)(uint32_t)mid1 +
                                     (uint64_t)(uint32_t)mid2 + (lo >> 32)) >>
                             32;

            uint64_t res = hi + (mid1 >> 32) + (mid2 >> 32) + carry;
            Ditem = res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhsu_vv") || (operation == "vmulhsu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Bitem < 0);
            uint64_t Vs1_lo = (uint32_t)(Aitem);
            uint64_t Vs1_hi = (uint64_t)(Aitem) >> 32;
            uint64_t Vs2_lo = (uint32_t)abs(Bitem);
            uint64_t Vs2_hi = (uint64_t)abs(Bitem) >> 32;

            uint64_t hi = Vs1_hi * Vs2_hi;
            uint64_t mid1 = Vs1_hi * Vs2_lo;
            uint64_t mid2 = Vs1_lo * Vs2_hi;
            uint64_t lo = Vs2_lo * Vs1_lo;
            uint64_t carry = ((uint64_t)(uint32_t)mid1 +
                                     (uint64_t)(uint32_t)mid2 + (lo >> 32)) >>
                             32;

            uint64_t res_lo =
                    (uint64_t)((uint64_t)((uint64_t)(uint32_t)mid1 << 32) +
                               (uint64_t)((uint64_t)(uint32_t)mid2 << 32) +
                               lo);

            uint64_t res_hi = hi + (mid1 >> 32) + (mid2 >> 32) + carry;

            if (negate) {
                if (res_hi > 0) {
                    Ditem = ~res_hi +
                            (((Aitem == 0) || (res_lo == 0)) ? 1 : 0);
                } else {
                    Ditem = (Aitem == 0) ? 0 : -1;
                }
            } else {
                Ditem = res_hi;
            }
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
        numMUL64_operations = numMUL64_operations.value() +
                              1; // number of 64-bit MUL operations
        numALU64_operations = numALU64_operations.value() -
                              1; // number of 64-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vdivu_vv") || (operation == "vdivu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint64_t)__UINT64_MAX__ :
                                        (uint64_t)((uint64_t)Bitem /
                                                   (uint64_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vremu_vv") || (operation == "vremu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint64_t)Bitem :
                                        (uint64_t)((uint64_t)Bitem %
                                                   (uint64_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") ||
            (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (unsigned long int)Bitem << Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %x << %x  = %x  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") ||
            (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (unsigned long int)Bitem >> Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsrl_wv") || (operation == "vnsrl_wx") ||
            (operation == "vnsrl_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint64_t)Bitem >> (Aitem & (64 * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (64 * 2 - 1), Ditem);
    }

    if ((operation == "vsra_vv") || (operation == "vsra_vx") ||
            (operation == "vsra_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsra_wv") || (operation == "vnsra_wx") ||
            (operation == "vnsra_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (__int128_t)(__uint128_t)(uint64_t)Bitem >>
                                (Aitem & (vsew * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (vsew * 2 - 1), Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") ||
            (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") ||
            (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") ||
            (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if (((operation == "vmerge_vv") || (operation == "vmerge_vx") ||
                (operation == "vmerge_vi")) &&
            (vm == 0)) {
        Ditem = (Mitem == 0) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if (((operation == "vmv_vv") || (operation == "vmv_vx") ||
                (operation == "vmv_vi")) &&
            (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %d  = %d\n", Aitem, Ditem);
    }

    if ((operation == "vmv1r_v") || (operation == "vmv2r_v") ||
            (operation == "vmv4r_v") || (operation == "vmv8r_v")) {
        Ditem = Bitem;
        DPRINTF(Datapath, "WB Instruction = %x  = %x\n", Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint64_t)Aitem < (uint64_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint64_t)Aitem > (uint64_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") ||
            (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") ||
            (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint64_t)((uint64_t)Bitem < (uint64_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") ||
            (operation == "vmsleu_vi")) {
        Ditem = (uint64_t)((uint64_t)Bitem <= (uint64_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") ||
            (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint64_t)((uint64_t)Bitem > (uint64_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d), old(%d) long int"
                "\n",
                Mitem, Dstitem);
    }

    return Ditem;
}

int
Datapath::compute_int_op(int Aitem, int Bitem, uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    int Ditem = 0;
    std::string operation = insn->getName();
    numALU32_operations =
            numALU32_operations.value() + 1; // number of 32-bit ALU operations

    if ((operation == "vsmul_vv") || (operation == "vsmul_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int64_t res = (int64_t)Bitem * (int64_t)Aitem;
            bool sat = false;
            Ditem = (res >> (32 - 1)) +
                    get_fixed_rounding_incr<int64_t>(res, 32 - 1,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = signed_saturation<int32_t, uint32_t>(32, Ditem, sat);
            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%d * %d)) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssra_vv") || (operation == "vssra_vx") ||
            (operation == "vssra_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int32_t shift_amount = (Aitem & mask(log2(32UL)));
            Ditem = (Bitem >> shift_amount) +
                    get_fixed_rounding_incr<int32_t>(Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath, "WB Instruction = roundoff_signed(%d, %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssrl_vv") || (operation == "vssrl_vx") ||
            (operation == "vssrl_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int32_t shift_amount = (Aitem & mask(log2(32UL)));
            Ditem = ((uint32_t)Bitem >> shift_amount) +
                    (uint32_t)get_fixed_rounding_incr<uint32_t>(
                            (uint32_t)Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d, %d) = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vaadd_vv") || (operation == "vaadd_vx")) {
        int64_t res = (int64_t)Bitem + (int64_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint32_t)get_fixed_rounding_incr<int64_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vaaddu_vv") || (operation == "vaaddu_vx")) {
        uint64_t res = (uint64_t)(uint32_t)Bitem + (uint64_t)(uint32_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint32_t)get_fixed_rounding_incr<uint64_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasub_vv") || (operation == "vasub_vx")) {
        int64_t res = (int64_t)Bitem - (int64_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint32_t)get_fixed_rounding_incr<int64_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasubu_vv") || (operation == "vasubu_vx")) {
        uint64_t res = (uint64_t)(uint32_t)Bitem - (uint64_t)(uint32_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint32_t)get_fixed_rounding_incr<uint64_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vmadc_vvm") || (operation == "vmadc_vxm") ||
            (operation == "vmadc_vim")) {
        Ditem = (((uint64_t)(uint32_t)Bitem + (uint64_t)(uint32_t)Aitem +
                         ((Mitem == 1) ? (uint64_t)(uint32_t)Mitem :
                                         (uint64_t)0)) > UINT32_MAX) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d + %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmadc_vv") || (operation == "vmadc_vx") ||
            (operation == "vmadc_vi")) {
        Ditem = ((uint64_t)(uint32_t)Bitem + (uint64_t)(uint32_t)Aitem) >
                                UINT32_MAX ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsbc_vvm") || (operation == "vmsbc_vxm")) {
        Ditem = ((uint64_t)(uint32_t)Bitem <
                        (uint64_t)(uint32_t)Aitem +
                                (uint64_t)((Mitem == 1) ? (uint32_t)Mitem :
                                                          0)) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d - %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmsbc_vv") || (operation == "vmsbc_vx")) {
        Ditem = ((uint32_t)Bitem < (uint32_t)Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmacc_vv") || (operation == "vmacc_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Bitem * Aitem + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vnmsac_vv") || (operation == "vnmsac_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Bitem * Aitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vmadd_vv") || (operation == "vmadd_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Dstitem * Aitem + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }
    if ((operation == "vnmsub_vv") || (operation == "vnmsub_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Dstitem * Aitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }

    if ((operation == "vadc_vvm") || (operation == "vadc_vxm") ||
            (operation == "vadc_vim")) {
        Ditem = Aitem + Bitem + Mitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d + %d = %d  \n", Aitem,
                Bitem, Mitem, Ditem);
    }

    if ((operation == "vsbc_vv") || (operation == "vsbc_vx") ||
            (operation == "vsbc_vi")) {
        Ditem = Bitem - Aitem - Mitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d - %d = %d  \n", Bitem,
                Aitem, Mitem, Ditem);
    }

    if ((operation == "vadd_vv") || (operation == "vadd_vx") ||
            (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") ||
            (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrsub_vx") || (operation == "vrsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem - Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsadd_vv") || (operation == "vsadd_vx") ||
            (operation == "vsadd_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_add<int32_t, uint32_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vsaddu_vv") || (operation == "vsaddu_vx") ||
            (operation == "vsaddu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_addu<uint32_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vssub_vv") || (operation == "vssub_vx") ||
            (operation == "vssub_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_sub<int32_t, uint32_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vssubu_vv") || (operation == "vssubu_vx") ||
            (operation == "vssubu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_subu<uint32_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmulh_vv") || (operation == "vmulh_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Aitem < 0) != (Bitem < 0);

            uint32_t Vs1_lo = (uint16_t)abs(Aitem);
            uint32_t Vs1_hi = (uint32_t)abs(Aitem) >> 16;
            uint32_t Vs2_lo = (uint16_t)abs(Bitem);
            uint32_t Vs2_hi = (uint32_t)abs(Bitem) >> 16;

            uint32_t hi = Vs1_hi * Vs2_hi;
            uint32_t mid1 = Vs1_hi * Vs2_lo;
            uint32_t mid2 = Vs1_lo * Vs2_hi;
            uint32_t lo = Vs2_lo * Vs1_lo;
            uint32_t carry = ((uint32_t)(uint16_t)mid1 +
                                     (uint32_t)(uint16_t)mid2 + (lo >> 16)) >>
                             16;

            uint32_t res = hi + (mid1 >> 16) + (mid2 >> 16) + carry;
            Ditem = negate ? ~res + (Aitem * Bitem == 0 ? 1 : 0) : res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhu_vv") || (operation == "vmulhu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            uint32_t Vs1_lo = (uint16_t)(Aitem);
            uint32_t Vs1_hi = (uint32_t)(Aitem) >> 16;
            uint32_t Vs2_lo = (uint16_t)(Bitem);
            uint32_t Vs2_hi = (uint32_t)(Bitem) >> 16;

            uint32_t hi = Vs1_hi * Vs2_hi;
            uint32_t mid1 = Vs1_hi * Vs2_lo;
            uint32_t mid2 = Vs1_lo * Vs2_hi;
            uint32_t lo = Vs2_lo * Vs1_lo;
            uint32_t carry = ((uint32_t)(uint16_t)mid1 +
                                     (uint32_t)(uint16_t)mid2 + (lo >> 16)) >>
                             16;

            uint32_t res = hi + (mid1 >> 16) + (mid2 >> 16) + carry;
            Ditem = res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhsu_vv") || (operation == "vmulhsu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Bitem < 0);
            uint32_t Vs1_lo = (uint16_t)(Aitem);
            uint32_t Vs1_hi = (uint32_t)(Aitem) >> 16;
            uint32_t Vs2_lo = (uint16_t)abs(Bitem);
            uint32_t Vs2_hi = (uint32_t)abs(Bitem) >> 16;

            uint32_t hi = Vs1_hi * Vs2_hi;
            uint32_t mid1 = Vs1_hi * Vs2_lo;
            uint32_t mid2 = Vs1_lo * Vs2_hi;
            uint32_t lo = Vs2_lo * Vs1_lo;
            uint32_t carry = ((uint32_t)(uint16_t)mid1 +
                                     (uint32_t)(uint16_t)mid2 + (lo >> 16)) >>
                             16;

            uint32_t res_lo =
                    (uint32_t)((uint32_t)((uint16_t)mid1 << 16) +
                               (uint32_t)((uint16_t)mid2 << 16) + lo);

            uint32_t res_hi = hi + (mid1 >> 16) + (mid2 >> 16) + carry;

            if (negate) {
                if (res_hi > 0) {
                    Ditem = ~res_hi +
                            (((Aitem == 0) || (res_lo == 0)) ? 1 : 0);
                } else {
                    Ditem = (Aitem == 0) ? 0 : -1;
                }
            } else {
                Ditem = res_hi;
            }
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d\n", Aitem, Bitem,
                Ditem);
        numMUL32_operations = numMUL32_operations.value() +
                              1; // number of 32-bit MUL operations
        numALU32_operations = numALU32_operations.value() -
                              1; // number of 32-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vdivu_vv") || (operation == "vdivu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint32_t)__UINT32_MAX__ :
                                        (uint32_t)((uint32_t)Bitem /
                                                   (uint32_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vremu_vv") || (operation == "vremu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint32_t)Bitem :
                                        (uint32_t)((uint32_t)Bitem %
                                                   (uint32_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") ||
            (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (unsigned int)Bitem << Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %x << %x  = %x  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") ||
            (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (unsigned int)Bitem >> Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsrl_wv") || (operation == "vnsrl_wx") ||
            (operation == "vnsrl_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint32_t)Bitem >> (Aitem & (32 * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (32 * 2 - 1), Ditem);
    }

    if ((operation == "vsra_vv") || (operation == "vsra_vx") ||
            (operation == "vsra_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsra_wv") || (operation == "vnsra_wx") ||
            (operation == "vnsra_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (int64_t)(uint64_t)(uint32_t)Bitem >>
                                (Aitem & (vsew * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (vsew * 2 - 1), Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") ||
            (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") ||
            (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") ||
            (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if (((operation == "vmerge_vv") || (operation == "vmerge_vx") ||
                (operation == "vmerge_vi")) &&
            (vm == 0)) {
        Ditem = (Mitem == 0) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if (((operation == "vmv_vv") || (operation == "vmv_vx") ||
                (operation == "vmv_vi")) &&
            (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %d  = %d\n", Aitem, Ditem);
    }

    if ((operation == "vmv1r_v") || (operation == "vmv2r_v") ||
            (operation == "vmv4r_v") || (operation == "vmv8r_v")) {
        Ditem = Bitem;
        DPRINTF(Datapath, "WB Instruction = %x  = %x\n", Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem > (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") ||
            (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") ||
            (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") ||
            (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") ||
            (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d), old(%d) int"
                "\n",
                Mitem, Dstitem);
    }
    return Ditem;
}

int16_t
Datapath::compute_int16_op(int16_t Aitem, int16_t Bitem, uint8_t Mitem,
        int16_t Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int16_t Ditem = 0;
    std::string operation = insn->getName();
    numALU16_operations =
            numALU16_operations.value() + 1; // number of 16-bit ALU operations

    if ((operation == "vsmul_vv") || (operation == "vsmul_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int32_t res = (int32_t)Bitem * (int32_t)Aitem;
            bool sat = false;
            Ditem = (res >> (16 - 1)) +
                    get_fixed_rounding_incr<int32_t>(res, 16 - 1,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = signed_saturation<int16_t, uint16_t>(16, Ditem, sat);
            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%d * %d)) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssra_vv") || (operation == "vssra_vx") ||
            (operation == "vssra_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int16_t shift_amount = (Aitem & mask(log2(16UL)));
            Ditem = (Bitem >> shift_amount) +
                    get_fixed_rounding_incr<int16_t>(Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath, "WB Instruction = roundoff_signed(%d, %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssrl_vv") || (operation == "vssrl_vx") ||
            (operation == "vssrl_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int16_t shift_amount = (Aitem & mask(log2(16UL)));
            Ditem = ((uint16_t)Bitem >> shift_amount) +
                    (uint16_t)get_fixed_rounding_incr<uint16_t>(
                            (uint16_t)Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d, %d) = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vaadd_vv") || (operation == "vaadd_vx")) {
        int32_t res = (int32_t)Bitem + (int32_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint16_t)get_fixed_rounding_incr<int32_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vaaddu_vv") || (operation == "vaaddu_vx")) {
        uint32_t res = (uint32_t)(uint16_t)Bitem + (uint32_t)(uint16_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint16_t)get_fixed_rounding_incr<uint32_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasub_vv") || (operation == "vasub_vx")) {
        int32_t res = (int32_t)Bitem - (int32_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint16_t)get_fixed_rounding_incr<int32_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasubu_vv") || (operation == "vasubu_vx")) {
        uint32_t res = (uint32_t)(uint16_t)Bitem - (uint32_t)(uint16_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint16_t)get_fixed_rounding_incr<uint32_t>(
                                        res, 1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vmadc_vvm") || (operation == "vmadc_vxm") ||
            (operation == "vmadc_vim")) {
        Ditem = (((uint32_t)(uint16_t)Bitem + (uint32_t)(uint16_t)Aitem +
                         ((Mitem == 1) ? (uint32_t)(uint16_t)Mitem :
                                         (uint32_t)0)) > UINT16_MAX) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d + %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmadc_vv") || (operation == "vmadc_vx") ||
            (operation == "vmadc_vi")) {
        Ditem = ((uint32_t)(uint16_t)Bitem + (uint32_t)(uint16_t)Aitem) >
                                UINT16_MAX ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsbc_vvm") || (operation == "vmsbc_vxm")) {
        Ditem = ((uint32_t)(uint16_t)Bitem <
                        (uint32_t)(uint16_t)Aitem +
                                (uint32_t)((Mitem == 1) ? (uint16_t)Mitem :
                                                          0)) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d - %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmsbc_vv") || (operation == "vmsbc_vx")) {
        Ditem = ((uint16_t)Bitem < (uint16_t)Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmacc_vv") || (operation == "vmacc_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Bitem * Aitem + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vnmsac_vv") || (operation == "vnmsac_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Bitem * Aitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vmadd_vv") || (operation == "vmadd_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Dstitem * Aitem + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }
    if ((operation == "vnmsub_vv") || (operation == "vnmsub_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Dstitem * Aitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }

    if ((operation == "vadc_vvm") || (operation == "vadc_vxm") ||
            (operation == "vadc_vim")) {
        Ditem = Aitem + Bitem + Mitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d + %d = %d  \n", Aitem,
                Bitem, Mitem, Ditem);
    }

    if ((operation == "vsbc_vv") || (operation == "vsbc_vx") ||
            (operation == "vsbc_vi")) {
        Ditem = Bitem - Aitem - Mitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d - %d = %d  \n", Bitem,
                Aitem, Mitem, Ditem);
    }

    if ((operation == "vadd_vv") || (operation == "vadd_vx") ||
            (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") ||
            (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrsub_vx") || (operation == "vrsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem - Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsadd_vv") || (operation == "vsadd_vx") ||
            (operation == "vsadd_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_add<int16_t, uint16_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vsaddu_vv") || (operation == "vsaddu_vx") ||
            (operation == "vsaddu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_addu<uint16_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vssub_vv") || (operation == "vssub_vx") ||
            (operation == "vssub_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_sub<int16_t, uint16_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vssubu_vv") || (operation == "vssubu_vx") ||
            (operation == "vssubu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_subu<uint16_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmulh_vv") || (operation == "vmulh_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Aitem < 0) != (Bitem < 0);

            uint16_t Vs1_lo = (uint8_t)abs(Aitem);
            uint16_t Vs1_hi = (uint16_t)abs(Aitem) >> 8;
            uint16_t Vs2_lo = (uint8_t)abs(Bitem);
            uint16_t Vs2_hi = (uint16_t)abs(Bitem) >> 8;

            uint16_t hi = Vs1_hi * Vs2_hi;
            uint16_t mid1 = Vs1_hi * Vs2_lo;
            uint16_t mid2 = Vs1_lo * Vs2_hi;
            uint16_t lo = Vs2_lo * Vs1_lo;
            uint16_t carry = ((uint16_t)(uint8_t)mid1 +
                                     (uint16_t)(uint8_t)mid2 + (lo >> 8)) >>
                             8;

            uint16_t res = hi + (mid1 >> 8) + (mid2 >> 8) + carry;
            Ditem = negate ? ~res + (Aitem * Bitem == 0 ? 1 : 0) : res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhu_vv") || (operation == "vmulhu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            uint16_t Vs1_lo = (uint8_t)(Aitem);
            uint16_t Vs1_hi = (uint16_t)(Aitem) >> 8;
            uint16_t Vs2_lo = (uint8_t)(Bitem);
            uint16_t Vs2_hi = (uint16_t)(Bitem) >> 8;

            uint16_t hi = Vs1_hi * Vs2_hi;
            uint16_t mid1 = Vs1_hi * Vs2_lo;
            uint16_t mid2 = Vs1_lo * Vs2_hi;
            uint16_t lo = Vs2_lo * Vs1_lo;
            uint16_t carry = ((uint16_t)(uint8_t)mid1 +
                                     (uint16_t)(uint8_t)mid2 + (lo >> 8)) >>
                             8;

            uint16_t res = hi + (mid1 >> 8) + (mid2 >> 8) + carry;
            Ditem = res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhsu_vv") || (operation == "vmulhsu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Bitem < 0);
            uint16_t Vs1_lo = (uint8_t)(Aitem);
            uint16_t Vs1_hi = (uint16_t)(Aitem) >> 8;
            uint16_t Vs2_lo = (uint8_t)abs(Bitem);
            uint16_t Vs2_hi = (uint16_t)abs(Bitem) >> 8;

            uint16_t hi = Vs1_hi * Vs2_hi;
            uint16_t mid1 = Vs1_hi * Vs2_lo;
            uint16_t mid2 = Vs1_lo * Vs2_hi;
            uint16_t lo = Vs2_lo * Vs1_lo;
            uint16_t carry = ((uint16_t)(uint8_t)mid1 +
                                     (uint16_t)(uint8_t)mid2 + (lo >> 8)) >>
                             8;

            uint16_t res_lo = (uint16_t)((uint16_t)((uint8_t)mid1 << 8) +
                                         (uint16_t)((uint8_t)mid2 << 8) + lo);

            uint16_t res_hi = hi + (mid1 >> 8) + (mid2 >> 8) + carry;

            if (negate) {
                if (res_hi > 0) {
                    Ditem = ~res_hi +
                            (((Aitem == 0) || (res_lo == 0)) ? 1 : 0);
                } else {
                    Ditem = (Aitem == 0) ? 0 : -1;
                }
            } else {
                Ditem = res_hi;
            }
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d\n", Aitem, Bitem,
                Ditem);
        numMUL16_operations = numMUL16_operations.value() +
                              1; // number of 16-bit MUL operations
        numALU16_operations = numALU16_operations.value() -
                              1; // number of 16-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vdivu_vv") || (operation == "vdivu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint16_t)__UINT16_MAX__ :
                                        (uint16_t)((uint16_t)Bitem /
                                                   (uint16_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vremu_vv") || (operation == "vremu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint16_t)Bitem :
                                        (uint16_t)((uint16_t)Bitem %
                                                   (uint16_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") ||
            (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint16_t)Bitem << Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %x << %x  = %x  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") ||
            (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint16_t)Bitem >> Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsrl_wv") || (operation == "vnsrl_wx") ||
            (operation == "vnsrl_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint16_t)Bitem >> (Aitem & (16 * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (16 * 2 - 1), Ditem);
    }

    if ((operation == "vsra_vv") || (operation == "vsra_vx") ||
            (operation == "vsra_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsra_wv") || (operation == "vnsra_wx") ||
            (operation == "vnsra_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (int32_t)(uint32_t)(uint16_t)Bitem >>
                                (Aitem & (vsew * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (vsew * 2 - 1), Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") ||
            (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") ||
            (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") ||
            (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if (((operation == "vmerge_vv") || (operation == "vmerge_vx") ||
                (operation == "vmerge_vi")) &&
            (vm == 0)) {
        Ditem = (Mitem == 0) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if (((operation == "vmv_vv") || (operation == "vmv_vx") ||
                (operation == "vmv_vi")) &&
            (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %d  = %d\n", Aitem, Ditem);
    }

    if ((operation == "vmv1r_v") || (operation == "vmv2r_v") ||
            (operation == "vmv4r_v") || (operation == "vmv8r_v")) {
        Ditem = Bitem;
        DPRINTF(Datapath, "WB Instruction = %x  = %x\n", Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem > (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") ||
            (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") ||
            (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") ||
            (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") ||
            (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d), old(%d) int16"
                "\n",
                Mitem, Dstitem);
    }
    return Ditem;
}

int8_t
Datapath::compute_int8_op(int8_t Aitem, int8_t Bitem, uint8_t Mitem,
        int8_t Dstitem, RiscvISA::VectorStaticInst* insn)
{
    int8_t Ditem = 0;
    std::string operation = insn->getName();
    numALU8_operations =
            numALU8_operations.value() + 1; // number of 8-bit ALU operations

    if ((operation == "vsmul_vv") || (operation == "vsmul_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int16_t res = (int16_t)Bitem * (int16_t)Aitem;
            bool sat = false;
            Ditem = (res >> (8 - 1)) +
                    get_fixed_rounding_incr<int16_t>(res, 8 - 1,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = signed_saturation<int8_t, uint8_t>(8, Ditem, sat);
            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%d * %d)) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssra_vv") || (operation == "vssra_vx") ||
            (operation == "vssra_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int8_t shift_amount = (Aitem & mask(log2(8UL)));
            Ditem = (Bitem >> shift_amount) +
                    get_fixed_rounding_incr<int8_t>(Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath, "WB Instruction = roundoff_signed(%d, %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vssrl_vv") || (operation == "vssrl_vx") ||
            (operation == "vssrl_vi")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            int8_t shift_amount = (Aitem & mask(log2(8UL)));
            Ditem = ((uint8_t)Bitem >> shift_amount) +
                    (uint8_t)get_fixed_rounding_incr<uint8_t>((uint8_t)Bitem,
                            shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d, %d) = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vaadd_vv") || (operation == "vaadd_vx")) {
        int16_t res = (int16_t)Bitem + (int16_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) + (uint8_t)get_fixed_rounding_incr<int16_t>(
                                             res, 1,
                                             (*xc)->readMiscReg(
                                                     RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vaaddu_vv") || (operation == "vaaddu_vx")) {
        uint16_t res = (uint16_t)(uint8_t)Bitem + (uint16_t)(uint8_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint8_t)get_fixed_rounding_incr<uint16_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d + %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasub_vv") || (operation == "vasub_vx")) {
        int16_t res = (int16_t)Bitem - (int16_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) + (uint8_t)get_fixed_rounding_incr<int16_t>(
                                             res, 1,
                                             (*xc)->readMiscReg(
                                                     RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_signed(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }
    if ((operation == "vasubu_vv") || (operation == "vasubu_vx")) {
        uint16_t res = (uint16_t)(uint8_t)Bitem - (uint16_t)(uint8_t)Aitem;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (res >> 1) +
                                (uint8_t)get_fixed_rounding_incr<uint16_t>(res,
                                        1,
                                        (*xc)->readMiscReg(
                                                RiscvISA::MISCREG_VXRM)) :
                        Dstitem;
        DPRINTF(Datapath,
                "WB Instruction = roundoff_unsigned(%d - %d)  = %d  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vmadc_vvm") || (operation == "vmadc_vxm") ||
            (operation == "vmadc_vim")) {
        Ditem = (((uint16_t)(uint8_t)Bitem + (uint16_t)(uint8_t)Aitem +
                         ((Mitem == 1) ? (uint16_t)(uint8_t)Mitem :
                                         (uint16_t)0)) > UINT8_MAX) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d + %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmadc_vv") || (operation == "vmadc_vx") ||
            (operation == "vmadc_vi")) {
        Ditem = ((uint16_t)(uint8_t)Bitem + (uint16_t)(uint8_t)Aitem) >
                                UINT8_MAX ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = carry_out(%d + %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmsbc_vvm") || (operation == "vmsbc_vxm")) {
        Ditem = ((uint16_t)(uint8_t)Bitem <
                        (uint16_t)(uint8_t)Aitem +
                                (uint16_t)((Mitem == 1) ? (uint8_t)Mitem :
                                                          0)) ?
                        1 :
                        0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d - %d) = %d  \n",
                Bitem, Aitem, Mitem, Ditem);
    }

    if ((operation == "vmsbc_vv") || (operation == "vmsbc_vx")) {
        Ditem = ((uint8_t)Bitem < (uint8_t)Aitem) ? 1 : 0;
        DPRINTF(Datapath, "WB Instruction = borrow_out(%d - %d) = %d  \n",
                Bitem, Aitem, Ditem);
    }

    if ((operation == "vmacc_vv") || (operation == "vmacc_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Bitem * Aitem + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vnmsac_vv") || (operation == "vnmsac_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Bitem * Aitem) + Dstitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Bitem,
                Aitem, Dstitem, Ditem);
    }
    if ((operation == "vmadd_vv") || (operation == "vmadd_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        Dstitem * Aitem + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }
    if ((operation == "vnmsub_vv") || (operation == "vnmsub_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        -(Dstitem * Aitem) + Bitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = -(%d * %d) + %d = %d  \n", Dstitem,
                Aitem, Bitem, Ditem);
    }

    if ((operation == "vadc_vvm") || (operation == "vadc_vxm") ||
            (operation == "vadc_vim")) {
        Ditem = Aitem + Bitem + Mitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d + %d = %d  \n", Aitem,
                Bitem, Mitem, Ditem);
    }

    if ((operation == "vsbc_vv") || (operation == "vsbc_vx") ||
            (operation == "vsbc_vi")) {
        Ditem = Bitem - Aitem - Mitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d - %d = %d  \n", Bitem,
                Aitem, Mitem, Ditem);
    }

    if ((operation == "vadd_vv") || (operation == "vadd_vx") ||
            (operation == "vadd_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem + Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsub_vv") || (operation == "vsub_vx") ||
            (operation == "vsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem - Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrsub_vx") || (operation == "vrsub_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem - Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vsadd_vv") || (operation == "vsadd_vx") ||
            (operation == "vsadd_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_add<int8_t, uint8_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vsaddu_vv") || (operation == "vsaddu_vx") ||
            (operation == "vsaddu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_addu<uint8_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d + %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vssub_vv") || (operation == "vssub_vx") ||
            (operation == "vssub_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_sub<int8_t, uint8_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vssubu_vv") || (operation == "vssubu_vx") ||
            (operation == "vssubu_vi")) {
        bool sat = false;
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        sat_subu<uint8_t>(Aitem, Bitem, sat) :
                        Dstitem;
        (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        DPRINTF(Datapath, "WB Instruction = %d - %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmulh_vv") || (operation == "vmulh_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Aitem < 0) != (Bitem < 0);

            uint8_t Vs1_lo = (uint8_t)(abs(Aitem) & 0xf);
            uint8_t Vs1_hi = (uint8_t)abs(Aitem) >> 4;
            uint8_t Vs2_lo = (uint8_t)(abs(Bitem) & 0xf);
            uint8_t Vs2_hi = (uint8_t)abs(Bitem) >> 4;

            uint8_t hi = Vs1_hi * Vs2_hi;
            uint8_t mid1 = Vs1_hi * Vs2_lo;
            uint8_t mid2 = Vs1_lo * Vs2_hi;
            uint8_t lo = Vs2_lo * Vs1_lo;
            uint8_t carry = ((uint8_t)(mid1 & 0xf) + (uint8_t)(mid2 & 0xf) +
                                    (lo >> 4)) >>
                            4;

            uint8_t res = hi + (mid1 >> 4) + (mid2 >> 4) + carry;
            Ditem = negate ? ~res + (Aitem * Bitem == 0 ? 1 : 0) : res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhu_vv") || (operation == "vmulhu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            uint8_t Vs1_lo = (uint8_t)((Aitem)&0xf);
            uint8_t Vs1_hi = (uint8_t)(Aitem) >> 4;
            uint8_t Vs2_lo = (uint8_t)((Bitem)&0xf);
            uint8_t Vs2_hi = (uint8_t)(Bitem) >> 4;

            uint8_t hi = Vs1_hi * Vs2_hi;
            uint8_t mid1 = Vs1_hi * Vs2_lo;
            uint8_t mid2 = Vs1_lo * Vs2_hi;
            uint8_t lo = Vs2_lo * Vs1_lo;
            uint8_t carry = ((uint8_t)(mid1 & 0xf) + (uint8_t)(mid2 & 0xf) +
                                    (lo >> 4)) >>
                            4;

            uint8_t res = hi + (mid1 >> 4) + (mid2 >> 4) + carry;
            Ditem = res;
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }
    if ((operation == "vmulhsu_vv") || (operation == "vmulhsu_vx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool negate = (Bitem < 0);
            uint8_t Vs1_lo = (uint8_t)((Aitem)&0xf);
            uint8_t Vs1_hi = (uint8_t)(Aitem) >> 4;
            uint8_t Vs2_lo = (uint8_t)(abs(Bitem) & 0xf);
            uint8_t Vs2_hi = (uint8_t)abs(Bitem) >> 4;

            uint8_t hi = Vs1_hi * Vs2_hi;
            uint8_t mid1 = Vs1_hi * Vs2_lo;
            uint8_t mid2 = Vs1_lo * Vs2_hi;
            uint8_t lo = Vs2_lo * Vs1_lo;
            uint8_t carry = ((uint8_t)(mid1 & 0xf) + (uint8_t)(mid2 & 0xf) +
                                    (lo >> 4)) >>
                            4;

            uint8_t res_lo = (uint8_t)((uint8_t)((mid1 & 0xf) << 4) +
                                       (uint8_t)((mid2 & 0xf) << 4) + lo);

            uint8_t res_hi = hi + (mid1 >> 4) + (mid2 >> 4) + carry;

            if (negate) {
                if (res_hi > 0) {
                    Ditem = ~res_hi +
                            (((Aitem == 0) || (res_lo == 0)) ? 1 : 0);
                } else {
                    Ditem = (Aitem == 0) ? 0 : -1;
                }
            } else {
                Ditem = res_hi;
            }
        } else {
            Ditem = Dstitem;
        }

        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d  \n", Aitem, Bitem,
                Ditem);
    }

    if ((operation == "vmul_vv") || (operation == "vmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Aitem * Bitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d * %d  = %d\n", Aitem, Bitem,
                Ditem);
        numMUL8_operations = numMUL8_operations.value() +
                             1; // number of 8-bit MUL operations
        numALU8_operations = numALU8_operations.value() -
                             1; // number of 8-bit ALU operations
    }

    if ((operation == "vdiv_vv") || (operation == "vdiv_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem / Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d\n", Bitem, Aitem,
                Ditem);
    }
    if ((operation == "vdivu_vv") || (operation == "vdivu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint8_t)__UINT8_MAX__ :
                                        (uint8_t)((uint8_t)Bitem /
                                                  (uint8_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d / %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vrem_vv") || (operation == "vrem_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem % Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d\n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vremu_vv") || (operation == "vremu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((Aitem == 0) ? (uint8_t)Bitem :
                                        (uint8_t)((uint8_t)Bitem %
                                                  (uint8_t)Aitem)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d mod %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsll_vv") || (operation == "vsll_vx") ||
            (operation == "vsll_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint8_t)Bitem << Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %x << %x  = %x  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vsrl_vv") || (operation == "vsrl_vx") ||
            (operation == "vsrl_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint8_t)Bitem >> Aitem :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsrl_wv") || (operation == "vnsrl_wx") ||
            (operation == "vnsrl_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (uint8_t)Bitem >> (Aitem & (8 * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (8 * 2 - 1), Ditem);
    }

    if ((operation == "vsra_vv") || (operation == "vsra_vx") ||
            (operation == "vsra_vi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ? Bitem >> Aitem :
                                                             Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vnsra_wv") || (operation == "vnsra_wx") ||
            (operation == "vnsra_wi")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (int16_t)(uint16_t)(uint8_t)Bitem >>
                                (Aitem & (vsew * 2 - 1)) :
                        Dstitem;
        DPRINTF(Datapath, "WB Instruction = %d >> %d  = %d  \n", Bitem,
                Aitem & (vsew * 2 - 1), Ditem);
    }

    if ((operation == "vand_vv") || (operation == "vand_vx") ||
            (operation == "vand_vi")) {
        Ditem = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x & 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vor_vv") || (operation == "vor_vx") ||
            (operation == "vor_vi")) {
        Ditem = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x | 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if ((operation == "vxor_vv") || (operation == "vxor_vx") ||
            (operation == "vxor_vi")) {
        Ditem = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = 0x%x ^ 0x%x  = 0x%x  \n", Bitem,
                Aitem, Ditem);
    }

    if (((operation == "vmerge_vv") || (operation == "vmerge_vx") ||
                (operation == "vmerge_vi")) &&
            (vm == 0)) {
        Ditem = (Mitem == 0) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d : %d  = %d\n", Aitem, Bitem,
                Ditem);
    }

    if (((operation == "vmv_vv") || (operation == "vmv_vx") ||
                (operation == "vmv_vi")) &&
            (vm == 1)) {
        Ditem = Aitem;
        DPRINTF(Datapath, "WB Instruction = %d  = %d\n", Aitem, Ditem);
    }

    if ((operation == "vmv1r_v") || (operation == "vmv2r_v") ||
            (operation == "vmv4r_v") || (operation == "vmv8r_v")) {
        Ditem = Bitem;
        DPRINTF(Datapath, "WB Instruction = %x  = %x\n", Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Min/Max Instructions
     *************************************************************************/

    if ((operation == "vmin_vv") || (operation == "vmin_vx")) {
        Ditem = (Aitem < Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vminu_vv") || (operation == "vminu_vx")) {
        Ditem = ((uint32_t)Aitem < (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = min :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmax_vv") || (operation == "vmax_vx")) {
        Ditem = (Aitem > Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    if ((operation == "vmaxu_vv") || (operation == "vmaxu_vx")) {
        Ditem = ((uint32_t)Aitem > (uint32_t)Bitem) ? Aitem : Bitem;
        DPRINTF(Datapath, "WB Instruction = %d , %d  = max :%d  \n", Aitem,
                Bitem, Ditem);
    }

    /**************************************************************************
     * Vector Integer Comparison Instructions
     *************************************************************************/
    if ((operation == "vmseq_vv") || (operation == "vmseq_vx") ||
            (operation == "vmseq_vi")) {
        Ditem = (Bitem == Aitem);
        DPRINTF(Datapath, "WB Instruction = %d == %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsne_vv") || (operation == "vmsne_vx") ||
            (operation == "vmsne_vi")) {
        Ditem = (Bitem != Aitem);
        DPRINTF(Datapath, "WB Instruction = %d != %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsltu_vv") || (operation == "vmsltu_vx")) {
        Ditem = (uint32_t)((uint32_t)Bitem < (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmslt_vv") || (operation == "vmslt_vx")) {
        Ditem = (Bitem < Aitem);
        DPRINTF(Datapath, "WB Instruction = %d < %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsleu_vv") || (operation == "vmsleu_vx") ||
            (operation == "vmsleu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem <= (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsle_vv") || (operation == "vmsle_vx") ||
            (operation == "vmsle_vi")) {
        Ditem = (Bitem <= Aitem);
        DPRINTF(Datapath, "WB Instruction = %d <= %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgtu_vx") || (operation == "vmsgtu_vi")) {
        Ditem = (uint32_t)((uint32_t)Bitem > (uint32_t)Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if ((operation == "vmsgt_vx") || (operation == "vmsgt_vi")) {
        Ditem = (Bitem > Aitem);
        DPRINTF(Datapath, "WB Instruction = %d > %d ? = %d  \n", Bitem, Aitem,
                Ditem);
    }

    if (vm == 0) {
        DPRINTF(Datapath,
                "WB Instruction is masked vm(%d), old(%d) int8"
                "\n",
                Mitem, Dstitem);
    }
    return Ditem;
}

/*
 * Integer Widening Instructions
 * WT: widening type (2*sew)
 * T: type (sew)
 * U: unsigned type
 */

template <typename WT, typename UWT, typename T, typename UT>
WT
Datapath::compute_widening_op(WT Aitem, WT Bitem, uint8_t Mitem, WT Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    WT Ditem = 0;
    std::string operation = insn->getName();

    // @TODO: add overload for 'operator<<' for __int128

    if ((operation == "vnclip_wv") || (operation == "vnclip_wx") ||
            (operation == "vnclip_wx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool sat = false;
            T shift_amount = (T)Aitem & mask(vsew);

            Ditem = (Bitem >> shift_amount) +
                    get_fixed_rounding_incr<WT>(Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = signed_saturation<WT, UWT>(vsew, Ditem, sat);

            // @TODO: add narrowing op
            // Currently re-use wideningCrossSew op
            Ditem = (WT)(Ditem & (UWT)mask(vsew)) |
                    (UWT)(Dstitem & ((UWT)mask(vsew) << vsew));

            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%#010x, %#010x)) = "
                "%#010x  \n",
                (T)Bitem, (T)Aitem, (T)Ditem);
    }

    if ((operation == "vnclipu_wv") || (operation == "vnclipu_wx") ||
            (operation == "vnclipu_wx")) {
        if ((vm == 1) || ((vm == 0) && (Mitem == 1))) {
            bool sat = false;
            T shift_amount = (T)Aitem & mask(vsew);

            Ditem = ((UWT)Bitem >> shift_amount) +
                    get_fixed_rounding_incr<UWT>((UWT)Bitem, shift_amount,
                            (*xc)->readMiscReg(RiscvISA::MISCREG_VXRM));

            Ditem = unsigned_saturation<UWT, UT>(vsew, Ditem, sat);

            // @TODO: add narrowing op
            // Currently re-use wideningCrossSew op
            Ditem = (UWT)(Ditem & (UWT)mask(vsew)) |
                    (UWT)(Dstitem & ((UWT)mask(vsew) << vsew));

            (*xc)->setMiscReg(RiscvISA::MISCREG_VXSAT, sat ? 1 : 0);
        } else {
            Ditem = Dstitem;
        }
        DPRINTF(Datapath,
                "WB Instruction = clip(roundoff_signed(%#010x, %#010x)) = "
                "%#010x  \n",
                (T)Bitem, (T)Aitem, (T)Ditem);
    }

    if ((operation == "vwadd_vv") || (operation == "vwadd_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)(T)Bitem + (WT)(T)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x + %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwaddu_vv") || (operation == "vwaddu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (UWT)(UT)Bitem + (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x + %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwadd_wv") || (operation == "vwadd_wx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)Bitem + (WT)(T)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x + %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwaddu_wv") || (operation == "vwaddu_wx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (UWT)Bitem + (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x + %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }

    if ((operation == "vwsub_vv") || (operation == "vwsub_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)(T)Bitem - (WT)(T)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x - %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwsubu_vv") || (operation == "vwsubu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (UWT)(UT)Bitem - (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x - %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwsub_wv") || (operation == "vwsub_wx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)Bitem - (WT)(T)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x - %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwsubu_wv") || (operation == "vwsubu_wx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (UWT)Bitem - (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x - %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }

    if ((operation == "vwmul_vv") || (operation == "vwmul_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)(T)Bitem * (WT)(T)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    if ((operation == "vwmulu_vv") || (operation == "vwmulu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (UWT)(UT)Bitem * (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }
    // signed(vs2), unsigned(vs1)
    if ((operation == "vwmulsu_vv") || (operation == "vwmulsu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        (WT)(T)Bitem * (UWT)(UT)Aitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x = %#018x  \n",
        // Bitem, Aitem, Ditem);
    }

    if ((operation == "vwmacc_vv") || (operation == "vwmacc_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((WT)(T)Bitem * (WT)(T)Aitem) + (WT)Dstitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x + %#018x = %#018x
        // \n", Bitem, Aitem, Dstitem, Ditem);
    }
    if ((operation == "vwmaccu_vv") || (operation == "vwmaccu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((UWT)(UT)Bitem * (UWT)(UT)Aitem) + (WT)Dstitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x + %#018x = %#018x
        // \n", Bitem, Aitem, Dstitem, Ditem);
    }
    // signed(vs1/rs1), unsigned(vs2)
    if ((operation == "vwmaccsu_vv") || (operation == "vwmaccsu_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((UWT)(UT)Bitem * (WT)(T)Aitem) + (WT)Dstitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x + %#018x = %#018x
        // \n", Bitem, Aitem, Dstitem, Ditem);
    }
    // signed(vs2), unsigned(rs1)
    // (operation == "vwmaccus_vv") is not provided in riscv-v-spec
    if ((operation == "vwmaccus_vx")) {
        Ditem = ((vm == 1) || ((vm == 0) && (Mitem == 1))) ?
                        ((WT)(T)Bitem * (UWT)(UT)Aitem) + (WT)Dstitem :
                        Dstitem;
        // DPRINTF(Datapath,"WB Instruction = %#018x * %#018x + %#018x = %#018x
        // \n", Bitem, Aitem, Dstitem, Ditem);
    }

    return Ditem;
}

/*
 * Integer Reductions :Missing cases
 */
int
Datapath::computeIntReduction(int accumInt, int Bitem, uint8_t Mitem)
{
    int reduction = 0;
    std::string operation = insn->getName();
    numALU32_operations =
            numALU32_operations.value() + 1; // number of 32-bit ALU operations

    if (operation == "vredsum_vs") {
        reduction = (vm == 1) ? accumInt + Bitem :
                    (Mitem)   ? accumInt + Bitem :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Acc= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmax_vs") {
        reduction = (vm == 1) ? ((accumInt > Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt > Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Max= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmaxu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint32_t)accumInt > (uint32_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint32_t)accumInt > (uint32_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Maxu= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmin_vs") {
        reduction = (vm == 1) ? ((accumInt < Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt < Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredminu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint32_t)accumInt < (uint32_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint32_t)accumInt < (uint32_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredand_vs") {
        reduction = (vm == 1) ? (accumInt & Bitem) :
                    (Mitem)   ? (accumInt & Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  and= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredor_vs") {
        reduction = (vm == 1) ? (accumInt | Bitem) :
                    (Mitem)   ? (accumInt | Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  or= %d\n", Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
        reduction = (vm == 1) ? (accumInt ^ Bitem) :
                    (Mitem)   ? (accumInt ^ Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  xor= %d\n", Bitem,
                reduction);
    }

    return reduction;
}

/*
 * Integer Reductions :Missing cases
 */
int8_t
Datapath::computeInt8Reduction(int8_t accumInt, int8_t Bitem, uint8_t Mitem)
{
    int8_t reduction = 0;
    std::string operation = insn->getName();
    numALU8_operations =
            numALU8_operations.value() + 1; // number of 8-bit ALU operations

    if (operation == "vredsum_vs") {
        reduction = (vm == 1) ? accumInt + Bitem :
                    (Mitem)   ? accumInt + Bitem :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Acc= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmax_vs") {
        reduction = (vm == 1) ? ((accumInt > Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt > Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Max= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmaxu_vs") {
        reduction =
                (vm == 1) ? (((uint8_t)accumInt > (uint8_t)Bitem) ? accumInt :
                                                                    Bitem) :
                (Mitem)   ? (((uint8_t)accumInt > (uint8_t)Bitem) ? accumInt :
                                                                    Bitem) :
                            accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Maxu= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmin_vs") {
        reduction = (vm == 1) ? ((accumInt < Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt < Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredminu_vs") {
        reduction =
                (vm == 1) ? (((uint8_t)accumInt < (uint8_t)Bitem) ? accumInt :
                                                                    Bitem) :
                (Mitem)   ? (((uint8_t)accumInt < (uint8_t)Bitem) ? accumInt :
                                                                    Bitem) :
                            accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredand_vs") {
        reduction = (vm == 1) ? (accumInt & Bitem) :
                    (Mitem)   ? (accumInt & Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  and= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredor_vs") {
        reduction = (vm == 1) ? (accumInt | Bitem) :
                    (Mitem)   ? (accumInt | Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  or= %d\n", Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
        reduction = (vm == 1) ? (accumInt ^ Bitem) :
                    (Mitem)   ? (accumInt ^ Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  xor= %d\n", Bitem,
                reduction);
    }

    return reduction;
}

/*
 * Integer Reductions :Missing cases
 */
int16_t
Datapath::computeInt16Reduction(int16_t accumInt, int16_t Bitem, uint8_t Mitem)
{
    int16_t reduction = 0;
    std::string operation = insn->getName();
    numALU16_operations =
            numALU16_operations.value() + 1; // number of 316bit ALU operations

    if (operation == "vredsum_vs") {
        reduction = (vm == 1) ? accumInt + Bitem :
                    (Mitem)   ? accumInt + Bitem :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Acc= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmax_vs") {
        reduction = (vm == 1) ? ((accumInt > Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt > Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Max= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmaxu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint16_t)accumInt > (uint16_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint16_t)accumInt > (uint16_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Maxu= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredmin_vs") {
        reduction = (vm == 1) ? ((accumInt < Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt < Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredminu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint16_t)accumInt < (uint16_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint16_t)accumInt < (uint16_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  Min= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredand_vs") {
        reduction = (vm == 1) ? (accumInt & Bitem) :
                    (Mitem)   ? (accumInt & Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  and= %d\n", Bitem,
                reduction);
    }

    if (operation == "vredor_vs") {
        reduction = (vm == 1) ? (accumInt | Bitem) :
                    (Mitem)   ? (accumInt | Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  or= %d\n", Bitem, reduction);
    }

    if (operation == "vredxor_vs") {
        reduction = (vm == 1) ? (accumInt ^ Bitem) :
                    (Mitem)   ? (accumInt ^ Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %d  xor= %d\n", Bitem,
                reduction);
    }

    return reduction;
}

/*
 * Integer Reductions :Missing cases
 */
long int
Datapath::computeLongIntReduction(
        long int accumInt, long int Bitem, uint8_t Mitem)
{
    long int reduction = 0;
    std::string operation = insn->getName();
    numALU64_operations =
            numALU64_operations.value() + 1; // number of 64-bit ALU operations

    if (operation == "vredsum_vs") {
        reduction = (vm == 1) ? accumInt + Bitem :
                    (Mitem)   ? accumInt + Bitem :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  Acc= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredmax_vs") {
        reduction = (vm == 1) ? ((accumInt > Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt > Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  Max= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredmaxu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint64_t)accumInt > (uint64_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint64_t)accumInt > (uint64_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  Maxu= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredmin_vs") {
        reduction = (vm == 1) ? ((accumInt < Bitem) ? accumInt : Bitem) :
                    (Mitem)   ? ((accumInt < Bitem) ? accumInt : Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  Min= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredminu_vs") {
        reduction =
                (vm == 1) ?
                        (((uint64_t)accumInt < (uint64_t)Bitem) ? accumInt :
                                                                  Bitem) :
                (Mitem) ? (((uint64_t)accumInt < (uint64_t)Bitem) ? accumInt :
                                                                    Bitem) :
                          accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  Min= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredand_vs") {
        reduction = (vm == 1) ? (accumInt & Bitem) :
                    (Mitem)   ? (accumInt & Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  and= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredor_vs") {
        reduction = (vm == 1) ? (accumInt | Bitem) :
                    (Mitem)   ? (accumInt | Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  or= %ld\n", Bitem,
                reduction);
    }

    if (operation == "vredxor_vs") {
        reduction = (vm == 1) ? (accumInt ^ Bitem) :
                    (Mitem)   ? (accumInt ^ Bitem) :
                                accumInt;
        DPRINTF(Datapath, " Reduction: Source %ld  xor= %ld\n", Bitem,
                reduction);
    }

    return reduction;
}

/*
 * Mask Operations
 */

long int
Datapath::computeLongMaskLogicalOp(
        bool Aitem, bool Bitem, RiscvISA::VectorStaticInst* insn)
{
    long int Ditem = 0;
    bool aux = 0;
    std::string operation = insn->getName();

    if ((operation == "vmand_mm")) {
        aux = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = %d & %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmnand_mm")) {
        aux = !(Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d & %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmandn_mm")) {
        aux = (Bitem & !Aitem);
        DPRINTF(Datapath, "WB Instruction = %d & !%d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmxor_mm")) {
        aux = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = %d ^ %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmor_mm")) {
        aux = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = %d | %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmnor_mm")) {
        aux = !(Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d | %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmorn_mm")) {
        aux = (Bitem | !Aitem);
        DPRINTF(Datapath, "WB Instruction = %d | !%d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmxnor_mm")) {
        aux = !(Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d | %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    Ditem = (uint64_t)aux;
    return Ditem;
}

int
Datapath::computeIntMaskLogicalOp(
        bool Aitem, bool Bitem, RiscvISA::VectorStaticInst* insn)
{
    int Ditem = 0;
    bool aux = 0;
    std::string operation = insn->getName();

    if ((operation == "vmand_mm")) {
        aux = (Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = %d & %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmnand_mm")) {
        aux = !(Bitem & Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d & %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmandn_mm")) {
        aux = (Bitem & !Aitem);
        DPRINTF(Datapath, "WB Instruction = %d & !%d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmxor_mm")) {
        aux = (Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = %d ^ %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmor_mm")) {
        aux = (Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = %d | %d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmnor_mm")) {
        aux = !(Bitem | Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d | %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmorn_mm")) {
        aux = (Bitem | !Aitem);
        DPRINTF(Datapath, "WB Instruction = %d | !%d = %d  \n", Bitem, Aitem,
                aux);
    }

    if ((operation == "vmxnor_mm")) {
        aux = !(Bitem ^ Aitem);
        DPRINTF(Datapath, "WB Instruction = !(%d | %d) = %d  \n", Bitem, Aitem,
                aux);
    }

    Ditem = (uint64_t)aux;
    return Ditem;
}

double
Datapath::compute_cvt_f_x_64_op(long int Bitem, uint8_t Mitem,
        long int Dstitem, RiscvISA::VectorStaticInst* insn)
{
    double Ditem = 0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 64-bit FP operations

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (double)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast (%d)  = %0.2lf\n", Bitem,
                Ditem);
    }

    if ((operation == "vfcvt_f_xu_v")) {
        Ditem = (double)(*(uint64_t*)&Bitem);
        DPRINTF(Datapath, "WB Instruction =  cast unsigned (%d)  = %0.2f\n",
                Bitem, Ditem);
    }

    return Ditem;
}

float
Datapath::compute_cvt_f_x_32_op(int Bitem, uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    float Ditem = 0;
    std::string operation = insn->getName();
    numFP32_operations =
            numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_f_x_v")) {
        Ditem = (float)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast (%d)  = %0.2f\n", Bitem,
                Ditem);
    }

    if ((operation == "vfcvt_f_xu_v")) {
        Ditem = (float)(*(uint32_t*)&Bitem);
        DPRINTF(Datapath, "WB Instruction =  cast unsigned (%d)  = %0.2f\n",
                Bitem, Ditem);
    }

    return Ditem;
}

long int
Datapath::compute_cvt_x_f_64_op(double Bitem, uint8_t Mitem, double Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    long int Ditem = 0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (long int)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast (%0.2lf) = %d\n", Bitem,
                Ditem);
    }

    if ((operation == "vfcvt_xu_f_v")) {
        Ditem = (unsigned long int)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast unsigned (%0.2lf) = %d\n",
                Bitem, Ditem);
    }

    return Ditem;
}

int
Datapath::compute_cvt_x_f_32_op(float Bitem, uint8_t Mitem, float Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    int Ditem = 0;
    std::string operation = insn->getName();
    numFP32_operations =
            numFP32_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfcvt_x_f_v")) {
        Ditem = (unsigned int)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast (%0.2f) = %d\n", Bitem,
                Ditem);
    }

    if ((operation == "vfcvt_xu_f_v")) {
        Ditem = (unsigned int)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast unsigned (%0.2f) = %d\n",
                Bitem, Ditem);
    }

    return Ditem;
}

double
Datapath::compute_cvt_f64_x32_op(int Bitem, uint8_t Mitem, int Dstitem,
        RiscvISA::VectorStaticInst* insn)
{
    double Ditem = 0.0;
    std::string operation = insn->getName();
    numFP64_operations =
            numFP64_operations.value() + 1; // number of 32-bit FP operations

    if ((operation == "vfwcvt_f_x_v")) {
        Ditem = (double)Bitem;
        DPRINTF(Datapath, "WB Instruction =  cast (%d)  = %0.2f\n", Bitem,
                Ditem);
    }

    return Ditem;
}

} // namespace RiscvISA

} // namespace gem5