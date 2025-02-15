# Copyright (c) 2020 Barcelona Supercomputing Center
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Cristobal Ramirez

from m5.params import *

from m5.objects.ClockedObject import *


class VectorRegister(ClockedObject):
    type = "VectorRegister"
    cxx_header = "cpu/vector_engine/vpu/register_file/vector_reg.hh"
    cxx_class = "gem5::RiscvISA::VectorRegister"

    num_lanes = Param.Unsigned("Number of vector lanes")
    num_regs = Param.Unsigned("Number of vector registers")
    mvl = Param.Unsigned("Maximum Vector Length - MVL")
    size = Param.Unsigned("Size of vector_reg in Bytes")
    lineSize = Param.Unsigned("how many bytes accessed per cycle")
    numPorts = Param.Unsigned("Independent Ports per vector_reg")
    accessLatency = Param.Cycles("Read/Write Latency to vector_reg")

    port = VectorSlavePort("Ports for other units to access the vector_reg")
