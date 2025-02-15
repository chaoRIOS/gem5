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

#include "arch/riscv/insts/vector.hh"

#include <sstream>

#include "arch/riscv/utility.hh"
#include "arch/riscv/insts/static_inst.hh"

using namespace std;

namespace gem5
{

namespace RiscvISA
{

string
RiscvVectorArithOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << " " << mnemonic << " ";
    ss << VectorRegNames[vd()];
    ss << VectorRegNames[vs1()] << ", ";
    ss << VectorRegNames[vs2()] ;
    if (vm()==0) {
        ss << ", " << "v0";
    }
    return ss.str();
}

string
RiscvVectorCfgOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << " " << mnemonic << " ";
    ss << IntRegNames[vd()] << ", ";
    ss << IntRegNames[vs1()] << ", ";
    if (getName() == "vsetvl") {
        ss << IntRegNames[vs2()] ;
    } else if (getName() == "vsetvli"){
        ss << vtype11() ;
    } else if (getName() == "vsetivli"){
        ss << vtype10() ;
    }
    return ss.str();
}

string
RiscvVectorMemOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << " " << mnemonic << " ";

    if (isLoad()) {
        ss << VectorRegNames[vd()];
    } else if (isStore()) {
        ss << VectorRegNames[vs3()];
    }
    ss << ",  (" << IntRegNames[vs1()] << ")";

    return ss.str();
}

string
RiscvVectorToScalarOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << ' ' << mnemonic << ' ';
    ss << IntRegNames[vd()] << ", " << VectorRegNames[vs2()];
    return ss.str();
}

string
RiscvVectorRegisterMoveOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << ' ' << mnemonic << ' ';
    ss << IntRegNames[vd()] << ", " << VectorRegNames[vs2()];
    return ss.str();
}

string
RiscvVectorIntegerWideningOp::generateDisassembly(Addr pc,
    const Loader::SymbolTable *symtab) const
{
    stringstream ss;
    ss << csprintf("0x%08x", machInst) << ' ' << mnemonic << ' ';
    ss << VectorRegNames[vd()];
    ss << VectorRegNames[vs1()] << ", ";
    ss << VectorRegNames[vs2()] ;
    return ss.str();
}


}

}