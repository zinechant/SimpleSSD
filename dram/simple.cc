/*
 * Copyright (C) 2017 CAMELab
 *
 * This file is part of SimpleSSD.
 *
 * SimpleSSD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SimpleSSD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SimpleSSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "dram/simple.hh"

#include "util/algorithm.hh"

namespace SimpleSSD {

namespace DRAM {

#define REFRESH_PERIOD 64000000000

SimpleDRAM::Stat::Stat() : count(0), size(0) {}

SimpleDRAM::SimpleDRAM(ConfigReader &p)
    : lastDRAMAccess(0), ignoreScheduling(false) {
  pStructure = p.getDRAMStructure();
  pTiming = p.getDRAMTiming();
  pageFetchLatency = pTiming->tRP + pTiming->tRAS;
  interfaceBandwidth = 2.0 * pStructure->busWidth * pStructure->chip *
                       pStructure->channel / 8.0 / pTiming->tCK;

  autoRefresh = allocate([this](uint64_t now) {
    lastDRAMAccess = MAX(lastDRAMAccess, now + pTiming->tRFC);

    schedule(autoRefresh, now + REFRESH_PERIOD);
  });

  schedule(autoRefresh, getTick() + REFRESH_PERIOD);
}

SimpleDRAM::~SimpleDRAM() {
  // DO NOTHING
}

void SimpleDRAM::updateDelay(uint64_t latency, uint64_t &tick) {
  if (tick > 0) {
    if (ignoreScheduling) {
      tick += latency;
    }
    else {
      if (lastDRAMAccess <= tick) {
        lastDRAMAccess = tick + latency;
      }
      else {
        lastDRAMAccess += latency;
      }

      tick = lastDRAMAccess;
    }
  }
}

void SimpleDRAM::setScheduling(bool enable) {
  ignoreScheduling = !enable;
}

bool SimpleDRAM::isScheduling() {
  return !ignoreScheduling;
}

void SimpleDRAM::read(void *, uint64_t size, uint64_t &tick) {
  uint64_t pageCount = (size > 0) ? (size - 1) / pStructure->pageSize + 1 : 0;
  uint64_t latency =
      (uint64_t)(pageCount * (pageFetchLatency +
                              pStructure->pageSize / interfaceBandwidth));
  updateDelay(latency, tick);

  // Stat Update
  readStat.count++;
  readStat.size += size;
}

void SimpleDRAM::write(void *, uint64_t size, uint64_t &tick) {
  uint64_t pageCount = (size > 0) ? (size - 1) / pStructure->pageSize + 1 : 0;
  uint64_t latency =
      (uint64_t)(pageCount * (pageFetchLatency +
                              pStructure->pageSize / interfaceBandwidth));
  updateDelay(latency, tick);

  // Stat Update
  writeStat.count++;
  writeStat.size += size;
}

void SimpleDRAM::getStatList(std::vector<Stats> &list, std::string prefix) {
  Stats temp;

  temp.name = prefix + "read.request_count";
  temp.desc = "Read request count";
  list.push_back(temp);

  temp.name = prefix + "read.bytes";
  temp.desc = "Read data size in byte";
  list.push_back(temp);

  temp.name = prefix + "write.request_count";
  temp.desc = "Write request count";
  list.push_back(temp);

  temp.name = prefix + "write.bytes";
  temp.desc = "Write data size in byte";
  list.push_back(temp);

  temp.name = prefix + "request_count";
  temp.desc = "Total request count";
  list.push_back(temp);

  temp.name = prefix + "bytes";
  temp.desc = "Total data size in byte";
  list.push_back(temp);
}

void SimpleDRAM::getStatValues(std::vector<double> &values) {
  values.push_back(readStat.count);
  values.push_back(readStat.size);
  values.push_back(writeStat.count);
  values.push_back(writeStat.size);
  values.push_back(readStat.count + writeStat.count);
  values.push_back(readStat.size + writeStat.size);
}

void SimpleDRAM::resetStatValues() {
  readStat = Stat();
  writeStat = Stat();
}

}  // namespace DRAM

}  // namespace SimpleSSD
