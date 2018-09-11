#include <utility>

#include <utility>

#include <utility>

//===----- IdemRegisterRenaming.cpp - Register regnaming after RA ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <llvm/PassSupport.h>
#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include <llvm/Support/Debug.h>
#include "LiveIntervalAnalysisIdem.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/Target/TargetData.h"

#include <algorithm>
#include <iterator>
#include <deque>

#define DEBUG_TYPE "reg-renaming"

using namespace llvm;

/// @author Jianping Zeng.
namespace {

  struct AntiDepPair {
    MachineOperand *use;
    MachineOperand *def;
    AntiDepPair(MachineOperand *_use, MachineOperand *_def) : use(_use), def(_def) {}
  };

  class RegisterRenaming : public MachineFunctionPass {
  public:
    static char ID;
    RegisterRenaming() : MachineFunctionPass(ID) {
      initializeRegisterRenamingPass(*PassRegistry::getPassRegistry());
    }
    virtual bool runOnMachineFunction(MachineFunction &MF) override;
    virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
      AU.addRequired<LiveIntervalAnalysisIdem>();
      AU.addRequired<MachineIdempotentRegions>();
      AU.setPreservesAll();
      MachineFunctionPass::getAnalysisUsage(AU);
    }
    const char *getPassName() const {
      return "Register Renaming for Idempotence pass";
    }

  private:
    void collectRefDefUseInfo(MachineInstr *mi, IdempotentRegion *region);
    bool shouldRename(AntiDepPair pair);
    unsigned choosePhysRegForRenaming(MachineOperand *use);
    unsigned tryChooseFreeRegister(LiveIntervalIdem &interval,
                                   const TargetRegisterClass &rc,
                                   BitVector &allocSet);
    bool availableOnRegClass(unsigned physReg, const TargetRegisterClass &rc);

    unsigned tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                      const TargetRegisterClass &rc,
                                      BitVector &allocSet);

    void spillOutInterval(LiveIntervalIdem *interval);

    // records all def registers by instr before current mi.
    std::map<MachineInstr*, std::set<unsigned>> prevDefRegs;
    // records all use registers of current mi and previous mi.
    std::map<MachineInstr*, std::set<MachineOperand*>> prevUseRegs;
    std::vector<AntiDepPair> antiDeps;
    const TargetInstrInfo *tii;
    const TargetRegisterInfo *tri;
    const MachineFunction *mf;
    LiveIntervalAnalysisIdem *li;
    MachineFrameInfo *mfi;
    MachineRegisterInfo *mri;
    const TargetData *td;
    std::deque<AntiDepPair> antiDepsList;
  };
}

char RegisterRenaming::ID = 0;

INITIALIZE_PASS_BEGIN(RegisterRenaming, "reg-renaming",
    "Register Renaming for Idempotence", false, false)
  INITIALIZE_PASS_DEPENDENCY(LiveIntervalAnalysisIdem)
  INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
INITIALIZE_PASS_END(RegisterRenaming, "reg-renaming",
                    "Register Renaming for Idempotence", false, false)

FunctionPass* llvm::createRegisterRenamingPass() {
  return new RegisterRenaming();
}

//=== Implementation for class RegisterRenaming.  ====//

template <class T>
static std::set<T> Union(std::set<T> &lhs, std::set<T> &rhs) {
  std::set<T> res;
  for (auto e : lhs)
    res.insert(e);
  for (auto e : rhs)
    res.insert(e);
  //res.insert(lhs.begin(), lhs.end());
  //res.insert(rhs.begin(), rhs.begin());
  return res;
}

template <class T>
static void intersect(std::set<T> &res, std::set<T> lhs, std::set<T> rhs) {
  for (T elt : lhs) {
    if (!rhs.count(elt))
      res.insert(elt);
  }
}

struct DefUseOfMI {
  std::set<MachineOperand*> *uses;
  std::set<unsigned> *defs;
};

std::map<MachineInstr*, DefUseOfMI*> map;

DefUseOfMI* getOrCreateDefUse(MachineInstr *mi,
                              std::set<MachineOperand*> *uses,
                              std::set<unsigned> *defs) {
  DefUseOfMI *&res = map[mi];
  if (!res)
    res = new DefUseOfMI{uses, defs};
  return res;
}

static void getDefUses(MachineInstr *mi, std::set<unsigned> *defs, std::set<MachineOperand *> *uses) {
  for (unsigned i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand *mo = &mi->getOperand(i);
    if (!mo || !mo->isReg() || !mo->getReg()) continue;
    unsigned &&reg = mo->getReg();
    assert(TargetRegisterInfo::isPhysicalRegister(reg));
    if (mo->isDef() && defs)
      defs->insert(reg);
    else if (mo->isUse() && uses)
      uses->insert(mo);
  }
}

bool contains(IdempotentRegion::inst_iterator begin,
              const IdempotentRegion::inst_iterator &end,
              MachineInstr *elt) {
  auto itr = std::move(begin);
  while(itr != end) {
    if (*itr == elt)
      return true;
    ++itr;
  }
  return false;
}

void RegisterRenaming::collectRefDefUseInfo(MachineInstr *mi,
    IdempotentRegion *region) {
  if (!mi) return;
  std::set<MachineOperand*> uses;
  std::set<unsigned> defs;
  getDefUses(mi, &defs, &uses);

  if (mi == &region->getEntry()) {
    prevDefRegs[mi] = std::set<unsigned>();
    prevUseRegs[mi] = uses;
  }
  else {
    // if the mi is the first mi of basic block with preds.
    if (mi == &mi->getParent()->front()) {
      MachineBasicBlock *mbb = mi->getParent();
      if (mbb->pred_empty()) {
        prevDefRegs[mi] = std::set<unsigned>();
        prevUseRegs[mi] = uses;
      }
      else {
        std::set<unsigned> &predDefs = prevDefRegs[mi];
        std::set<MachineOperand*> &predUses = prevUseRegs[mi];

        auto itr = mbb->pred_begin();
        auto end = mbb->pred_end();
        for (; itr != end; ++itr) {
          MachineBasicBlock *pred = *itr;
          MachineInstr *predMI = &pred->back();
          if (!contains(region->inst_begin(), region->inst_end(), predMI))
            continue;

          std::set<MachineOperand*> localUses;
          std::set<unsigned> localDefs;
          getDefUses(predMI, &localDefs, &localUses);

          predDefs = Union(localDefs, prevDefRegs[predMI]);
          predUses = Union(localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    }
    else {
      // otherwise
      std::set<unsigned> localPrevDefs;
      MachineBasicBlock::iterator miItr(mi);
      --miItr;
      getDefUses(miItr, &localPrevDefs, 0);
      prevDefRegs[mi] = Union(prevDefRegs[miItr], localPrevDefs);
      prevUseRegs[mi] = Union(prevUseRegs[miItr], uses);
    }
  }

  if (defs.empty())
    return;
  unsigned defReg = *defs.begin();
  std::set<MachineOperand*> &prevUses = prevUseRegs[mi];
  for (MachineOperand *mo : prevUses) {
    if (mo->isReg() && mo->getReg() == defReg &&
        !prevDefRegs[mo->getParent()].count(mo->getReg())) {
      antiDeps.emplace_back(mo, &mi->getOperand(0));
    }
  }
}

bool RegisterRenaming::shouldRename(AntiDepPair pair) {
  auto use = pair.use;
  MachineInstr *useMI = use->getParent();
  std::set<unsigned> &defs = prevDefRegs[useMI];
  return !defs.count(use->getReg());
}

void RegisterRenaming::spillOutInterval(LiveIntervalIdem *interval) {
  /// FIXME, I don't  know whether spilling code in here will arise no idempotence
  int frameIndex = INT_MIN;
  for (auto itr = interval->usepoint_begin(), end = interval->usepoint_end();
      itr != end; ++itr) {
    MachineOperand *mo = itr->mo;
    MachineInstr *mi = mo->getParent();
    assert(mo->isReg());
    const TargetRegisterClass *rc = tri->getMinimalPhysRegClass(mo->getReg());
    if (mo->isDef()) {
      frameIndex = mfi->CreateSpillStackObject(rc->getSize(), rc->getAlignment());
      MachineBasicBlock::instr_iterator insertPos = mi;
      tii->storeRegToStackSlot(*mi->getParent(), ++insertPos, mo->getReg(), true, frameIndex, rc, tri);
    }
    else if (mo->isUse()) {
       assert(frameIndex != INT_MIN);
      tii->loadRegFromStackSlot(*mi->getParent(), mi, mo->getReg(), frameIndex, rc, tri);
    }
  }
}

bool RegisterRenaming::availableOnRegClass(unsigned physReg,
                                           const TargetRegisterClass &rc) {
  return tri->getMinimalPhysRegClass(physReg)->hasSubClassEq(&rc) && !mri->isLiveIn(physReg);
}

unsigned RegisterRenaming::tryChooseFreeRegister(LiveIntervalIdem &interval,
                                                 const TargetRegisterClass &rc,
                                                 BitVector &allocSet) {
  for (int physReg = allocSet.find_first(); physReg > 0; physReg = allocSet.find_next(physReg)) {
    if (li->intervals.count(physReg)) {
      LiveIntervalIdem *itrv = li->intervals[physReg];
      if (!itrv->intersects(&interval)) {
        // we only consider those live interval which doesn't interfere with current
        // interval.
        // No matching in register class should be ignored.
        // Avoiding LiveIn register(such as argument register).
        if (!availableOnRegClass(physReg, rc))
          continue;

        //FIXME, we should check whether anti-dependence will occurs after inserting a move instr.
        return physReg;
      }
    }
    else if (availableOnRegClass(physReg, rc)){
      // current physReg is free, so return it.
      return static_cast<unsigned int>(physReg);
    }
  }
  return 0;
}

unsigned RegisterRenaming::tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                                    const TargetRegisterClass &rc,
                                                    BitVector &allocSet) {
  // choose an interval to be evicted out memory, and insert spilling code as
  // appropriate.
  unsigned costMax = INT_MAX;
  LiveIntervalIdem *targetInter = 0;
  for (auto itr = li->interval_begin(), end = li->interval_end(); itr != end; ++itr) {
    assert(interval.intersects(itr->second) &&
        "should not have interval doesn't interfere with current interval");

    unsigned reg = itr->second->reg;
    if (mri->isLiveIn(reg)) continue;

    if (itr->second->costToSpill < costMax) {
      costMax = itr->second->costToSpill;
      targetInter = itr->second;
    }
  }
  // no proper interval found to be spilled out.
  if (!targetInter) return 0;
  spillOutInterval(targetInter);
  return targetInter->reg;
}

unsigned RegisterRenaming::choosePhysRegForRenaming(MachineOperand *use) {
  auto rc = tri->getMinimalPhysRegClass(use->getReg());
  auto allocSet = tri->getAllocatableSet(*mf, rc);
  MachineInstr *mi = use->getParent();
  MachineBasicBlock::instr_iterator pos(mi);

  LiveIntervalIdem *interval = new LiveIntervalIdem;
  auto to = li->getIndex(mi);
  auto from = to - 1;
  interval->addRange(from, to);    // add an interval for a temporal move instr.

  unsigned freeReg = tryChooseFreeRegister(*interval, *rc, allocSet);
  if (freeReg) {
    // obtains a free register used for move instr.
    return freeReg;
  }
  freeReg = tryChooseBlockedRegister(*interval, *rc, allocSet);
  assert(freeReg && "can not to rename the specified register!");
  interval->reg = freeReg;
  li->addNewInterval(freeReg, interval);
  return freeReg;
}

bool RegisterRenaming::runOnMachineFunction(MachineFunction &MF) {
  mf = &MF;
  tii = MF.getTarget().getInstrInfo();
  tri = MF.getTarget().getRegisterInfo();
  li = getAnalysisIfAvailable<LiveIntervalAnalysisIdem>();
  mfi = MF.getFrameInfo();
  mri = &MF.getRegInfo();
  td = MF.getTarget().getTargetData();

  // Step#1: Collects regions
  MachineIdempotentRegions &regions = getAnalysis<MachineIdempotentRegions>();
  auto itr = regions.begin();
  auto end = regions.end();
  for (; itr != end; ++itr) {
    IdempotentRegion* region = *itr;
    auto mbb = region->mbb_begin();
    auto end = region->mbb_end();
    for (; mbb != end; ++mbb) {
      auto mi = mbb.getMBB().instr_begin();
      auto mie = mbb.getMBB().instr_end();
      // Step#2: visits register operand of each machine instr in the program sequence.
      for (; mi != mie; ++mi) {
        // Step#3: collects reg definition information.
        // Step#4: collects reg uses information.
        collectRefDefUseInfo(mi, region);
      }
    }
  }

  bool changed = false;

  // Step#5: checks if we should rename the defined register according to usd-def,
  //         If it is, construct a pair of use-def reg pair.

  // remove some equivalent anti-dependence, like
  //    |----|
  //    v    v
  //   r0 = r0 + r1
  //       /
  //     /
  //   r0 = r0 + 1
  for (size_t i = 0, e = antiDeps.size(); i < e; i++) {
    auto target = antiDeps[i];
    for (size_t j = i + 1; j < e; ++j) {
      auto pair = antiDeps[j];
      if (pair.use == target.use && pair.def != target.def &&
          pair.def->getReg() == target.def->getReg()) {
        // we can remove pair from antiDep list.
        antiDeps[j] = antiDeps[e-1];
        antiDeps.pop_back();
        --j;
        --e;
      }
    }
  }

  antiDepsList.insert(antiDepsList.end(), antiDeps.begin(), antiDeps.end());
  while (!antiDepsList.empty()) {
    AntiDepPair pair = antiDepsList.front();
    antiDepsList.pop_front();

    if (shouldRename(pair)) {
      // Step#6: choose a physical register for renaming.
      // Step#7: if there is no free physical register, using heuristic method to
      //         spill out a interval.
      auto useMI = pair.use->getParent();
      unsigned phyReg = choosePhysRegForRenaming(pair.use);
      assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
      assert(phyReg != pair.use->getReg());

      // Step#8: substitute the old reg with phyReg.
      unsigned oldReg = pair.use->getReg();
      pair.use->setReg(phyReg);

      MachineBasicBlock *parent = useMI->getParent();
      MachineBasicBlock::iterator itr2(useMI);

      // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
      bool needsBoundary = !tii->isIdemBoundary(--itr2);

      // Step#9: insert a splitting boundary as necessary.
      if (needsBoundary) {
        // The new region starts at I.
        tii->emitIdemBoundary(*parent, itr2);
        ++itr2;
      }
      assert(tii->isIdemBoundary(itr2) && "the mi at inserted position must be a splitting boundary!");
      // Step#10: insert a move instruction before splitting boundary instr.
      tii->copyPhysReg(*parent, itr2, DebugLoc(), phyReg, oldReg, false);

      DEBUG(pair.use->getParent()->getParent()->dump());
      changed = true;
    }
  }
  return changed;
}