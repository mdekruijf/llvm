#include <deque>
#include "LiveIntervalAnalysisIdem.h"

using namespace llvm;

LiveRangeIdem LiveRangeIdem::EndMarker(INT32_MAX, INT32_MAX, 0);

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
  if (first == &LiveRangeIdem::EndMarker || to < first->end)
    first = insertRangeBefore(from, to, first);
  else {
    LiveRangeIdem *r = first;
    while (r != &LiveRangeIdem::EndMarker) {
      if (to >= r->end)
        r = r->next;
      else
        break;
    }
    insertRangeBefore(from, to, r);
  }
}

void LiveIntervalIdem::print(llvm::raw_ostream &OS, const TargetRegisterInfo &tri) {
  OS << (TargetRegisterInfo::isPhysicalRegister(reg) ?
         tri.getName(reg) : ("%vreg" + reg));
  LiveRangeIdem *r = first;
  while (r && r != &LiveRangeIdem::EndMarker) {
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
  OS << "]";
}

bool LiveIntervalIdem::isLiveAt(unsigned pos) {
  if (pos <= first->start || pos >= last->end)
    return false;

  LiveRangeIdem *itr = first;
  while (itr != &LiveRangeIdem::EndMarker) {
    if (itr->contains(pos))
      return true;
    itr = itr->next;
  }
  return false;
}

LiveRangeIdem *LiveIntervalIdem::insertRangeBefore(unsigned from, unsigned to, LiveRangeIdem *cur) {
  assert(cur == &LiveRangeIdem::EndMarker || cur->end == INT32_MAX ||
      (cur->next && to < cur->next->start && "Not inserting at begining of interval"));
  assert(from <= cur->end && "Not inserting at begining of interval");
  if (cur->start <= to) {
    assert(cur != &LiveRangeIdem::EndMarker && "First range must not be EndMarker");
    cur->start = std::min(from, cur->start);
    cur->end = std::max(to, cur->end);
  } else {
    if (cur == last)
      cur = last = new LiveRangeIdem(from, to, cur);
    else {
      LiveRangeIdem *r = new LiveRangeIdem(from, to, last->next);
      last->next = r;
      last = r;
    }
  }
  return cur;
}

bool UsePoint::operator< (const UsePoint rhs) const{
  if (id < rhs.id) return true;
  if (id > rhs.id) return false;

  MachineOperand *rhsMO = rhs.mo;
  assert(mo->getParent() == rhsMO->getParent() && "must within same machine instr");
  MachineInstr *mi = rhsMO->getParent();
  int idx1 = -1, idx2 = -1;
  for (int i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand &MO = mi->getOperand(i);
    if (!mo->isReg() || !mo->getReg())
      continue;

    unsigned reg = mo->getReg();
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
  idx2MI.reserve(totalMIs);
  unsigned index = 0;
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

      out.insert(liveOuts[num].begin(), liveOuts[num].end());
      changed = out != liveOuts[num];
      if (changed)
        liveOuts[num] = out;

      std::set<unsigned> in = liveOuts[num];
      diff(in, liveKill[num]);
      in.insert(liveGen[num].begin(), liveGen[num].end());
      changed = in != liveIns[num];
      if (changed)
        liveIns[num] = in;
    }
  } while (changed);
}

void LiveIntervalAnalysisIdem::handleRegisterDef(unsigned reg, MachineOperand *mo, unsigned start) {
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
          const TargetRegisterClass *rc = tri->getMinimalPhysRegClass(mo.getReg());
          allocatableRegs = tri->getAllocatableSet(*mf, rc);
          // skip unallocatable register.
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
        handleRegisterDef(reg, op, num);
        if (TargetRegisterInfo::isPhysicalRegister(reg)) {
          const unsigned *subregs = tri->getSubRegisters(reg);
          if (subregs)
            for (; *subregs; ++subregs)
              if (!mi->modifiesRegister(*subregs, tri))
                handleRegisterDef(*subregs, op, num);
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
        if (li->first->end == num)
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
  int *numIncomingBranches = new int[size];
  MachineDominatorTree *dt = getAnalysisIfAvailable<MachineDominatorTree>();
  assert(dt);
  unsigned idx = 0;
  for (MachineFunction::iterator itr = MF.begin(), end = MF.end();
        itr != end; ++itr) {
    unsigned numPreds = std::distance(itr->pred_begin(), itr->pred_end());
    for (auto predItr = itr->pred_begin(), predEnd = itr->pred_end(); predItr != predEnd; ++predItr) {
      if (dt->dominates(&*itr, *predItr))
        --numPreds;
    }
    numIncomingBranches[idx] = numPreds;
    ++idx;
  }

  // Step #1: compute block order.
  std::vector<MachineBasicBlock *> sequence;
  std::deque<MachineBasicBlock *> worklist;
  worklist.push_back(&MF.front());
  while (!worklist.empty()) {
    MachineBasicBlock *curMBB = worklist.front();
    worklist.pop_front();
    sequence.push_back(curMBB);

    for (
    auto itr = curMBB->pred_begin(), end = curMBB->pred_end();
    itr != end; ++itr) {
      auto succ = *itr;
      --numIncomingBranches[succ->getNumber()];
      if (!numIncomingBranches[succ->getNumber()])
        worklist.push_back(succ);
    }
  }

  delete[] numIncomingBranches;

  // Step#2: computes local data flow information.
  std::vector<std::set<unsigned> > liveGen(size);
  std::vector<std::set<unsigned> > liveKill(size);
  computeLocalLiveSet(sequence, liveGen, liveKill);

  // Step#3: compute global live set.
  liveIns.reserve(size);
  liveOuts.reserve(size);
  computeGlobalLiveSet(sequence, liveIns, liveOuts, liveGen, liveKill);

  // Step $4: number the machine instrs.
  numberMachineInstr(sequence);

  // Step#5: build intervals.
  buildIntervals(sequence, liveOuts);

  // Step#6: TODO compute spilling weight for each interval.
  return false;
}

void LiveIntervalAnalysisIdem::addNewInterval(unsigned int reg,
                                              LiveIntervalIdem *pIdem) {
  intervals.insert(std::pair<unsigned, LiveIntervalIdem*>(reg, pIdem));
}
