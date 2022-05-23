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


#ifndef __ARCH_RISCV_VECTOR_INSTS_HH__
#define __ARCH_RISCV_VECTOR_INSTS_HH__

#include <string>

#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/types.hh"

#include "arch/riscv/insts/vector_static_inst.hh"
#include "cpu/static_inst.hh"

#include "base/loader/symtab.hh"
#include "cpu/reg_class.hh"

namespace gem5
{

namespace RiscvISA
{
/*
 * Vector Arithmetic Instructions
 */
class RiscvVectorArithOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorArithOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };

/*
 * Vector Configuration Instructions
 */
class RiscvVectorCfgOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorCfgOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
/*
 * Vector Memory Instructions
 */
class RiscvVectorMemOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorMemOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
/*
 * Move data to scalar core instructions
 */
class RiscvVectorToScalarOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorToScalarOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };
/*
 * Whole Vector Register Move instructions
 */
class RiscvVectorRegisterMoveOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorRegisterMoveOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };

/*
 * Vector Integer Widending instructions
 */
class RiscvVectorIntegerWideningOp : public RiscvVectorInsn
    {
      public:
        RiscvVectorIntegerWideningOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass) :
            RiscvVectorInsn(mnem, _machInst, __opClass){}

        std::string generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const;
    };

}

}

#endif // __ARCH_RISCV_VECTOR_INSTS_HH__