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

        VectorMemUnit::VectorMemUnit(const VectorMemUnitParams &params) : SimObject(SimObjectParams(params)), occupied(false),
                                                                          memReader(params.memReader), memReader_addr(params.memReader_addr),
                                                                          memWriter(params.memWriter)
        {
        }

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

            assert(!occupied);
            occupied = true;

            vectorwrapper = &vector_wrapper;

            uint64_t vl_count = vl;
            uint64_t vsew = vectorwrapper->vector_config->get_vtype_sew(vtype);

            /* destination data type size in bytes */
            uint8_t DST_SIZE = vsew / 8;
            assert(DST_SIZE != 0);

            uint8_t mop = insn.mop();
            bool unit_strided = (mop == 0);
            bool indexed_unordered = (mop == 1);
            bool strided = (mop == 2);
            bool indexed_ordered = (mop == 3);

            bool indexed = indexed_unordered || indexed_ordered;

            uint8_t lumop = insn.lumop();
            uint8_t sumop = insn.sumop();

            bool whole_register = (mop == 0) && ((lumop == 0x8) || (sumop == 0x8));

            uint64_t stride = (strided) ? src2 : 1;

            uint8_t nf = insn.nf();

            std::stringstream mem_mop;
            if (indexed_ordered)
            {
                mem_mop << "indexed_ordered";
            }
            else if (indexed_unordered)
            {
                mem_mop << "indexed_unordered";
            }
            else if (unit_strided)
            {
                mem_mop << "unit_strided";
            }
            else if (strided)
            {
                mem_mop << "strided (" << stride << ")";
            }
            else
            {
                mem_mop << " ";
            }

            // If vl_count == 0 then callback, means that the VL = 0
            if (vl_count == 0)
            {
                occupied = false;
                done_callback(NoFault);
                return;
            }

            // @TODO: add vstart & vl checking

            uint64_t mem_addr_dest;
            bool location0;
            uint64_t mem_addr_data;
            bool location;

            uint64_t mvl_bits =
                vectorwrapper->vector_config->get_max_vector_length_bits(vtype);
            uint64_t mvl_elem =
                vectorwrapper->vector_config->get_max_vector_length_elem(vtype);

            if (mvl_elem < vl_count)
            {
                panic("Vector register width %d is insufficient "
                      "for vl(%d) * sew(%d)",
                      mvl_bits, vl_count, vsew);
            }

            if (insn.isLoad())
            {
                mem_addr_dest = (uint64_t)dyn_insn->get_renamed_dst() * mvl_bits / 8;
                location0 = 1; // 1 Vecor Register

                DPRINTF(VectorMemUnit,
                        "Vector Load %s to Register "
                        "v%d @ %#010x, vl:%lu\n",
                        mem_mop.str(), dyn_insn->get_renamed_dst(),
                        (uint64_t)dyn_insn->get_renamed_dst() * mvl_bits / 8,
                        vl_count);

                // NOTE: need to initialize the writer BEFORE the reader!

                // Note: Writer needs max_vl_elem count for 0-filling instructions
                memWriter->initialize(vector_wrapper,
                                      whole_register ? nf * mvl_elem : mvl_elem, DST_SIZE,
                                      mem_addr_dest, 0, 1, location0, xc,
                                      [done_callback, this](bool done)
                                      {
                                          if (done)
                                          {
                                              this->occupied = false;
                                              done_callback(NoFault);
                                          }
                                      });

                mem_addr_data = src1;
                location = 0;
                DPRINTF(VectorMemUnit,
                        "Vector Load %s from Base Memory Addrs: 0x%lx\n",
                        mem_mop.str(), mem_addr_data);

                if (indexed)
                {
                    uint8_t INDEX_SIZE = 0;
                    switch (insn.func3())
                    {
                    case 0:
                        // ei8
                        INDEX_SIZE = 1;
                        break;

                    case 5:
                        // ei16
                        INDEX_SIZE = 2;
                        break;

                    case 6:
                        // ei32
                        INDEX_SIZE = 4;
                        break;

                    case 7:
                        // ei64
                        INDEX_SIZE = 8;
                        break;

                    default:
                        break;
                    }

                    uint64_t mem_addr_index =
                        (uint64_t)dyn_insn->get_renamed_src2() * mvl_bits / 8;

                    DPRINTF(VectorMemUnit,
                            "Vector Load Index from Vs2 v%d @"
                            "0x%lx\n",
                            dyn_insn->get_renamed_src2(), mem_addr_index);

                    memReader_addr->initialize(vector_wrapper, vl_count, INDEX_SIZE,
                                               mem_addr_index, 0, 1, location0, xc,
                                               [INDEX_SIZE, this](
                                                   uint8_t *data, uint8_t size, bool done)
                                               {
                                                   uint8_t *ndata = new uint8_t[INDEX_SIZE];
                                                   memcpy(ndata, data, INDEX_SIZE);
                                                   if (INDEX_SIZE == 8)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Data index addr 0x%x \n",
                                                               *(uint64_t *)ndata);
                                                   }
                                                   if (INDEX_SIZE == 4)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Data index addr 0x%x \n",
                                                               *(uint32_t *)ndata, done);
                                                   }
                                                   if (INDEX_SIZE == 2)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Data index addr 0x%x \n",
                                                               *(uint16_t *)ndata);
                                                   }
                                                   if (INDEX_SIZE == 1)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Data index addr 0x%x \n",
                                                               *(uint8_t *)ndata);
                                                   }
                                                   this->memReader->queueData(ndata);
                                                   this->memReader->setIndexWidth(INDEX_SIZE);
                                                   delete[] data;
                                               });
                }

                memReader->initialize(vector_wrapper,
                                      whole_register ? nf * mvl_elem : vl_count, DST_SIZE,
                                      mem_addr_data, mop, stride, location, xc,
                                      [mvl_elem, vl_count, DST_SIZE, whole_register, this](
                                          uint8_t *data, uint8_t size, bool done)
                                      {
                                          uint8_t *ndata = new uint8_t[DST_SIZE];
                                          memcpy(ndata, data, DST_SIZE);
                                          if (DST_SIZE == 8)
                                          {
                                              DPRINTF(VectorMemUnit, "queue Data 0x%x \n",
                                                      *(uint64_t *)ndata);
                                          }
                                          if (DST_SIZE == 4)
                                          {
                                              DPRINTF(VectorMemUnit, "queue Data 0x%x \n",
                                                      *(uint32_t *)ndata);
                                          }
                                          if (DST_SIZE == 2)
                                          {
                                              DPRINTF(VectorMemUnit, "queue Data 0x%x \n",
                                                      *(uint16_t *)ndata);
                                          }
                                          if (DST_SIZE == 1)
                                          {
                                              DPRINTF(VectorMemUnit, "queue Data 0x%x \n",
                                                      *(uint8_t *)ndata);
                                          }
                                          this->memWriter->queueData(ndata);
                                          delete[] data;
                                          if (!whole_register)
                                          {
                                              // Fill remaining elements with 0
                                              if (done)
                                              {
                                                  int zero_count = mvl_elem - vl_count;
                                                  uint8_t *ZeroData =
                                                      (uint8_t *)malloc(zero_count * DST_SIZE);
                                                  uint64_t zero_data = 0;
                                                  for (int i = 0; i < zero_count; i++)
                                                  {
                                                      memcpy(ZeroData + (i * DST_SIZE),
                                                             (uint8_t *)&zero_data, DST_SIZE);
                                                  }
                                                  for (int i = 0; i < zero_count; i++)
                                                  {
                                                      uint8_t *ndata = new uint8_t[DST_SIZE];
                                                      memcpy(ndata, ZeroData + (i * DST_SIZE),
                                                             DST_SIZE);
                                                      this->memWriter->queueData(ndata);
                                                      if (DST_SIZE == 8)
                                                      {
                                                          DPRINTF(VectorMemUnit,
                                                                  "queue Data "
                                                                  "0x%x \n",
                                                                  *(uint64_t *)ndata);
                                                      }
                                                      if (DST_SIZE == 4)
                                                      {
                                                          DPRINTF(VectorMemUnit,
                                                                  "queue Data "
                                                                  "0x%x \n",
                                                                  *(uint32_t *)ndata);
                                                      }
                                                      if (DST_SIZE == 2)
                                                      {
                                                          DPRINTF(VectorMemUnit,
                                                                  "queue Data "
                                                                  "0x%x \n",
                                                                  *(uint16_t *)ndata);
                                                      }
                                                      if (DST_SIZE == 1)
                                                      {
                                                          DPRINTF(VectorMemUnit,
                                                                  "queue Data "
                                                                  "0x%x \n",
                                                                  *(uint8_t *)ndata);
                                                      }
                                                  }
                                                  delete[] ZeroData;
                                              }
                                          }
                                      });
            }
            else if (insn.isStore())
            {
                mem_addr_dest = src1;
                location0 = 0; // 0 = Memoria

                DPRINTF(VectorMemUnit, "Vector Store %s to Memory Addrs: 0x%lx\n",
                        mem_mop.str(), mem_addr_dest);

                // NOTE: need to initialize the writer BEFORE the reader!
                memWriter->initialize(vector_wrapper, vl_count, DST_SIZE,
                                      mem_addr_dest, mop, stride, location0, xc,
                                      [done_callback, this](bool done)
                                      {
                                          if (done)
                                          {
                                              this->occupied = false;
                                              done_callback(NoFault);
                                          }
                                      });

                if (indexed)
                {
                    uint64_t mem_addr_index =
                        (uint64_t)dyn_insn->get_renamed_src2() * mvl_bits / 8;
                    location0 = 1; // 1 Vecor Register
                    DPRINTF(VectorMemUnit,
                            "Vector Store: Index from vector register v%d\n",
                            dyn_insn->get_renamed_src2());
                    memReader_addr->initialize(vector_wrapper, vl_count, DST_SIZE,
                                               mem_addr_index, 0, 1, location0, xc,
                                               [DST_SIZE, this](uint8_t *data, uint8_t size, bool done)
                                               {
                                                   uint8_t *ndata = new uint8_t[DST_SIZE];
                                                   memcpy(ndata, data, DST_SIZE);
                                                   if (DST_SIZE == 8)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Addrs index addr 0x%x \n",
                                                               *(uint64_t *)ndata);
                                                   }
                                                   if (DST_SIZE == 4)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Addrs index addr 0x%x \n",
                                                               *(uint32_t *)ndata);
                                                   }
                                                   if (DST_SIZE == 2)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Addrs index addr 0x%x \n",
                                                               *(uint16_t *)ndata);
                                                   }
                                                   if (DST_SIZE == 1)
                                                   {
                                                       DPRINTF(VectorMemUnit,
                                                               "queue Addrs index addr 0x%x \n",
                                                               *(uint8_t *)ndata);
                                                   }
                                                   this->memWriter->queueAddrs(ndata);
                                                   delete[] data;
                                               });
                }

                mem_addr_data = (uint64_t)dyn_insn->get_renamed_src3() * mvl_bits / 8;
                location = 1;
                DPRINTF(VectorMemUnit, "Vector Store: data from vector register v%d\n",
                        dyn_insn->get_renamed_src3());

                memReader->initialize(vector_wrapper, vl_count, DST_SIZE,
                                      mem_addr_data, 0, 1, location, xc,
                                      [DST_SIZE, this](uint8_t *data, uint8_t size, bool done)
                                      {
                                          uint8_t *ndata = new uint8_t[DST_SIZE];
                                          memcpy(ndata, data, DST_SIZE);
                                          // if (DST_SIZE==8) {
                                          //     DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint64_t
                                          //     *)ndata);
                                          // }
                                          // if (DST_SIZE==4) {
                                          //     DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint32_t
                                          //     *)ndata);
                                          // }
                                          // if (DST_SIZE==2) {
                                          //     DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint16_t
                                          //     *)ndata);
                                          // }
                                          // if (DST_SIZE==1) {
                                          //     DPRINTF(VectorMemUnit,"queue Data %X \n",*(uint8_t
                                          //     *)ndata);
                                          // }

                                          this->memWriter->queueData(ndata);
                                          delete[] data;
                                      });
            }
            else
            {
                panic("invalid Memory Operation type, insn=%#h \n", insn.machInst);
            }
        }

    } // namespace RiscvISA

} // namespace gem5
