#include <deque>
#include "LiveIntervalAnalysisIdem.h"
#include "IdemUtil.h"

using namespace llvm;

RangeIterator LiveRangeIdem::intersectsAt(LiveRangeIdem *r2) {
  assert(r2);
  RangeIterator itr1(this), itr2(r2), end(0);
  while (true) {
    if (itr1->start < itr2->start) {
      if (itr1->end <= itr2->start) {
        ++itr1;
        if (itr1 == end)
          return end;
      }
      else
        return itr2;
    }
    else {
      if (itr1->start == itr2->start)
        return itr1;
      // Otherwise, r1.start > r2.start <--> r2.start < r1.start
      if (itr2->end <= itr1->start) {
        ++itr2;
        if (itr2 == end)
          return end;
      }
      else
        return itr1;
    }
  }
}

LiveIntervalIdem::~LiveIntervalIdem() {
  if (first) {
    LiveRangeIdem * cur = first;
    while (cur) {
      LiveRangeIdem * next = cur->next;
      delete cur;
      cur = next;
    }
  }
}

void LiveIntervalIdem::addRange(unsigned from, unsigned to) {
  assert(from <= to && "Invalid range!");
  if (first == nullptr || to < first->end)
    insertRangeBefore(from, to, first);
  else {
    LiveRangeIdem **r = &first;
    while (*r != nullptr) {
      if (to >= (*r)->end)
        r = &(*r)->next;
      else
        break;
    }
    insertRangeBefore(from, to, *r);
  }
}

void LiveIntervalIdem::print(llvm::raw_ostream &OS, const TargetRegisterInfo &tri) {
  OS << (TargetRegisterInfo::isPhysicalRegister(reg) ?
         tri.getName(reg) : ("%vreg" + reg));
  LiveRangeIdem *r = first;
  while (r && r != nullptr) {
    r->dump();
    OS << ",";
    r = r->next;
  }

  OS << " Use points: [";
  unsigned long i = 0, size = usePoints.size();
  for (UsePoint up : usePoints) {
    OS << up.id;
    if (i < size - 1)
      OS << ",";
    ++i;
  }
  OS << "]\n";
}

bool LiveIntervalIdem::isLiveAt(unsigned pos) {
  if (pos <= first->start || pos >= last->end)
    return false;

  LiveRangeIdem *itr = first;
  while (itr != nullptr) {
    if (itr->contains(pos))
      return true;
    itr = itr->next;
  }
  return false;
}

bool LiveIntervalIdem::intersects(LiveIntervalIdem *cur) {
  assert(cur);
  if (cur->beginNumber() > endNumber())
    return false;

  return intersectAt(cur)!=end();
}
RangeIterator LiveIntervalIdem::intersectAt(LiveIntervalIdem *li) {
  return first->intersectsAt(li->first);
}

void LiveIntervalIdem::insertRangeBefore(unsigned from, unsigned to, LiveRangeIdem *&cur) {
  assert((cur == nullptr || to < cur->end) && "Not inserting at begining of interval");
  if (!cur) {
    //assert(!last);
    //assert(cur == first && "current node should be the first!");
    last = cur = new LiveRangeIdem(from, to, nullptr);
    return;
  }
  if (cur->start <= to) {
    assert(cur != nullptr && "First range must not be EndMarker");
    cur->start = std::min(from, cur->start);
    cur->end = std::max(to, cur->end);
  }
  else {
      cur = new LiveRangeIdem(from, to, cur);
  }
}

bool UsePoint::operator< (const UsePoint rhs) const{
  if (id < rhs.id) return true;
  if (id > rhs.id) return false;

  MachineOperand *rhsMO = rhs.mo;
  assert(mo->getParent() == rhsMO->getParent() && "must within same machine instr");
  MachineInstr *mi = rhsMO->getParent();
  int idx1 = -1, idx2 = -1;
  for (size_t i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand &MO = mi->getOperand(i);
    if (!mo->isReg() || !mo->getReg())
      continue;

    if (&MO == mo)
      idx1 = i;
    else if (&MO == rhsMO)
      idx2 = i;
  }
  assert(idx1 != -1 && idx2 != -1);
  return idx1 < idx2;
}

char LiveIntervalAnalysisIdem::ID = 0;
INITIALIZE_PASS_BEGIN(LiveIntervalAnalysisIdem, "live-interval-idem",
                      "Live Interval computing for Register Renaming", false, false)
  INITIALIZE_PASS_DEPENDENCY(LiveVariables)
  INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
  INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(LiveIntervalAnalysisIdem, "live-interval-idem",
                    "Live Interval computing for Register Renaming", false, false)

void LiveIntervalAnalysisIdem::computeLocalLiveSet(
    std::vector<MachineBasicBlock *> &sequence,
    std::vector<std::set<unsigned> > &liveGen,
    std::vector<std::set<unsigned> > liveKill) {
  for (auto mbb : sequence) {
    auto itr = mbb->rbegin();
    auto end = mbb->rend();
    for (; itr != end; ++itr) {
      MachineInstr *mi = &*itr;
      for (int j = mi->getNumOperands() - 1; j >= 0; --j) {
        MachineOperand &mo = mi->getOperand(j);
        if (!mo.isReg())
          continue;

        unsigned reg = mo.getReg();
        if (mo.isUse()) {
          if (!liveKill[mbb->getNumber()].count(reg))
            liveGen[mbb->getNumber()].insert(reg);
        } else if (mo.isDef())
          liveKill[mbb->getNumber()].insert(reg);
      }
    }
  }
}

void LiveIntervalAnalysisIdem::numberMachineInstr(std::vector<MachineBasicBlock *> &sequence) {
  if (sequence.empty())
    return;
  unsigned totalMIs = 0;
  for (auto mbb : sequence)
    totalMIs += mbb->size();

  idx2MI.clear();
  mi2Idx.clear();
  idx2MI.resize(totalMIs+1);

  // starts from 4 in the case of we will insert a interval before first instr.
  unsigned index = NUM;
  for (auto mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto end = mbb->instr_end();
    for (; mi != end; ++mi) {
      mi2Idx[&*mi] = index;
      idx2MI[index / NUM] = &*mi;
      index += NUM;
    }
  }
}

template<class T>
void diff(std::set<T> &res, std::set<T> &rhs) {
  auto itr = res.begin();
  auto end = res.end();
  for (; itr != end; ++itr) {
    if (rhs.count(*itr)) {
      res.erase(itr);
      --itr;
    }
  }
}

void LiveIntervalAnalysisIdem::computeGlobalLiveSet(
    std::vector<MachineBasicBlock *> &sequence,
    std::vector<std::set<unsigned> > &liveIns,
    std::vector<std::set<unsigned> > &liveOuts,
    std::vector<std::set<unsigned> > &liveGen,
    std::vector<std::set<unsigned> > &liveKill) {
  bool changed;
  do {
    changed = false;
    auto itr = sequence.rbegin();
    auto end = sequence.rend();
    for (; itr != end; ++itr) {
      auto mbb = *itr;
      int num = mbb->getNumber();
      std::set<unsigned> out;
      if (!mbb->succ_empty()) {
        auto succItr = mbb->succ_begin();
        auto succEnd = mbb->succ_end();
        for (; succItr != succEnd; ++succItr) {
          auto &set = liveIns[(*succItr)->getNumber()];
          out.insert(set.begin(), set.end());
        }
      }
      std::set<unsigned> lo = liveOuts[num];
      out.insert(lo.begin(), lo.end());
      changed = out != liveOuts[num];
      if (changed)
        liveOuts[num] = out;

      std::set<unsigned> in = liveOuts[num];
      diff(in, liveKill[num]);
      in.insert(liveGen[num].begin(), liveGen[num].end());
      auto &res = liveIns[num];
      if (in != res) {
        liveIns[num] = in;
        changed = true;
      }
    }
  } while (changed);
}

void LiveIntervalAnalysisIdem::handleRegisterDef(unsigned reg,
    MachineOperand *mo,
    unsigned start,
    unsigned end) {
  LiveIntervalIdem *&li = intervals[reg];
  if (!li) {
    li = new LiveIntervalIdem();
    li->reg = reg;
  }
  assert(li && "must be no null");
  if (mo->isDead()) {
    li->addRange(start, start + 1);
    li->addUsePoint(start, mo);
  } else {
    LiveRangeIdem *&lr = li->first;
    if (!lr)
      li->addRange(start, end);
    else
      lr->start = start;
    li->addUsePoint(start, mo);
  }
}

void LiveIntervalAnalysisIdem::buildIntervals(
    std::vector<MachineBasicBlock *> &sequence,
    std::vector<std::set<unsigned> > &liveOuts) {
  intervals.clear();
  auto itr = sequence.rbegin();
  auto end = sequence.rend();
  for (; itr != end; ++itr) {
    MachineBasicBlock *mbb = *itr;
    if (mbb->empty())
      continue;

    assert(mi2Idx.count(&mbb->front()));
    assert(mi2Idx.count(&mbb->back()));
    unsigned blockFrom = mi2Idx[&mbb->front()];
    unsigned blockTo = mi2Idx[&mbb->back()] + NUM;
    std::set<unsigned> &set = liveOuts[mbb->getNumber()];
    for (unsigned reg : set) {
      LiveIntervalIdem *&li = intervals[reg];
      if (!li) {
        li = new LiveIntervalIdem();
        li->reg = reg;
      }

      li->addRange(blockFrom, blockTo);
    }

    for (auto mi = mbb->instr_rbegin(), miEnd = mbb->instr_rend(); mi != miEnd; ++mi) {
      unsigned num = mi2Idx[&*mi];
      std::vector<MachineOperand *> uses, defs;
      for (unsigned moIdx = 0, sz = mi->getNumOperands(); moIdx < sz; ++moIdx) {
        MachineOperand &mo = mi->getOperand(moIdx);
        if (mo.isReg() && mo.getReg()) {
          // const TargetRegisterClass *rc = tri->getMinimalPhysRegClass(mo.getReg());
          allocatableRegs = tri->getAllocatableSet(*mf);
          // skip unallocable register.
          if (TargetRegisterInfo::isPhysicalRegister(mo.getReg()) &&
              !allocatableRegs[mo.getReg()])
            continue;
          if (mo.isDef())
            defs.push_back(&mo);
          else if (mo.isUse())
            uses.push_back(&mo);
        }
      }

      // handle defined registers.
      for (MachineOperand *op : defs) {
        unsigned reg = op->getReg();
        handleRegisterDef(reg, op, num, blockTo);
        if (TargetRegisterInfo::isPhysicalRegister(reg)) {
          const unsigned *subregs = tri->getSubRegisters(reg);
          if (subregs)
            for (; *subregs; ++subregs)
              if (!mi->modifiesRegister(*subregs, tri))
                handleRegisterDef(*subregs, op, num, blockTo);
        }
      }

      // handle use registers.
      for (MachineOperand *op : uses) {
        unsigned reg = op->getReg();
        LiveIntervalIdem *&li = intervals[reg];
        if (!li) {
          li = new LiveIntervalIdem();
          li->reg = reg;
        }

        assert(li);
        li->addRange(blockFrom, num);
        // extends the use to cross current instruction.
        if (li->first->end == num && li->first->start > 0)
          --li->first->start;

        li->addUsePoint(num, op);
      }
    }
  }
}

bool LiveIntervalAnalysisIdem::runOnMachineFunction(MachineFunction &MF) {
  mf = &MF;
  tri = MF.getTarget().getRegisterInfo();
  unsigned size = MF.getNumBlockIDs();
  dt = getAnalysisIfAvailable<MachineDominatorTree>();
  loopInfo = getAnalysisIfAvailable<MachineLoopInfo>();
  assert(dt);

  // Step #1: compute block order.
  std::vector<MachineBasicBlock *> sequence;
  computeReversePostOrder(MF, *dt, sequence);

  // Step#2: computes local data flow information.
  std::vector<std::set<unsigned> > liveGen(size);
  std::vector<std::set<unsigned> > liveKill(size);
  computeLocalLiveSet(sequence, liveGen, liveKill);

  // Step#3: compute global live set.
  liveIns.resize(size);
  liveOuts.resize(size);
  computeGlobalLiveSet(sequence, liveIns, liveOuts, liveGen, liveKill);

  // Step $4: number the machine instrs.
  numberMachineInstr(sequence);

  // Step#5: build intervals.
  buildIntervals(sequence, liveOuts);

  // Step#6:
  weightLiveInterval();

  // Dump some useful information for it to review the correctness
  // of this transformation.
  IDEM_DEBUG(dump(sequence););
  return false;
}

void LiveIntervalAnalysisIdem::insertOrCreateInterval(unsigned int reg,
                                                      LiveIntervalIdem *pIdem) {
  if (intervals.count(reg)) {
    auto itr = pIdem->begin();
    auto end = pIdem->end();
    for (; itr != end; ++itr)
      intervals[reg]->addRange(itr->start, itr->end);
    // accumulate cost to be spilled.
    intervals[reg]->costToSpill += pIdem->costToSpill;
    // add use points to the interval.
    intervals[reg]->usePoints.insert(pIdem->usepoint_begin(), pIdem->usepoint_end());
  }
  else {
    intervals.insert(std::pair<unsigned, LiveIntervalIdem *>(reg, pIdem));
  }
}

void LiveIntervalAnalysisIdem::weightLiveInterval() {
  // loop over all live intervals to compute spill cost.
  auto itr = interval_begin();
  auto end = interval_end();
  for(; itr != end; ++itr) {
    // Weight each use point by it's loop nesting deepth.
    unsigned cost = 0;
    for (auto &up : itr->second->usePoints) {
      MachineBasicBlock *mbb = up.mo->getParent()->getParent();
      if (MachineLoop *ml = loopInfo->getLoopFor(mbb)) {
        cost += 10*ml->getLoopDepth();
      }
      else
        cost += 1;
    }
    itr->second->costToSpill = cost;
  }
}

void LiveIntervalAnalysisIdem::dump(std::vector<MachineBasicBlock *> &sequence) {
  llvm::errs()<<"\nMachine instruction and Slot: \n";
  for (auto mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto end = mbb->instr_end();
    for (; mi != end; ++mi) {
      llvm::errs()<<mi2Idx[mi]<<":";
      mi->dump();
    }
  }

  llvm::errs()<<"\nLive Interval for Idempotence:\n";
  auto li = interval_begin();
  auto end = interval_end();
  for (; li != end; ++li) {
    li->second->dump(*const_cast<TargetRegisterInfo*>(mf->getTarget().getRegisterInfo()));
    llvm::errs()<<"\n";
  }
}
