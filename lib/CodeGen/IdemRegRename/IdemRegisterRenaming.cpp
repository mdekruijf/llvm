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
static void Union(std::set<T> &res, std::set<T> &lhs, std::set<T> &rhs) {
  res.insert(lhs.begin(), lhs.end());
  res.insert(rhs.begin(), rhs.begin());
}

template <class T>
static void intersect(std::set<T> &res, std::set<T> lhs, std::set<T> rhs) {
  for (T elt : lhs) {
    if (!rhs.count(elt))
      res.insert(elt);
  }
}

static void getDefUses(MachineInstr *mi, std::set<unsigned> *defs, std::set<MachineOperand *> *uses) {
  for (unsigned i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand *mo = &mi->getOperand(i);
    if (!mo->isReg() || !mo->getReg()) continue;
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
  std::set<MachineOperand*> uses;
  std::set<unsigned> defs;
  getDefUses(mi, &defs, &uses);

  if (defs.empty() || defs.size() != 1)
    return;

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
          MachineInstr *predMI = &*pred->getLastNonDebugInstr();
          if (contains(region->inst_begin(), region->inst_end(), predMI))
            continue;

          std::set<MachineOperand*> localUses;
          std::set<unsigned> localDefs;
          getDefUses(predMI, &localDefs, &localUses);

          Union(predDefs, localDefs, prevDefRegs[predMI]);
          Union(predUses, localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    }
    // otherwise
    std::set<unsigned> localPrevDefs;
    MachineBasicBlock::iterator miItr(mi);
    --miItr;
    getDefUses(miItr, &localPrevDefs, 0);
    Union(prevDefRegs[mi], prevDefRegs[miItr], localPrevDefs);
    Union(prevUseRegs[mi], prevUseRegs[miItr], uses);
  }

  unsigned defReg = *defs.begin();
  std::set<MachineOperand*> &prevUses = prevUseRegs[mi];
  for (MachineOperand *mo : prevUses) {
    if (mo->isReg() && mo->getReg() == defReg &&
        prevDefRegs[mo->getParent()].count(mo->getReg())) {
      antiDeps.push_back(AntiDepPair(mo, &mi->getOperand(0)));
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

unsigned RegisterRenaming::tryChooseFreeRegister(LiveIntervalIdem &interval,
                                                 const TargetRegisterClass &rc,
                                                 BitVector &allocSet) {
  for (auto itr = li->interval_begin(), end = li->interval_end(); itr != end; ++itr) {
    if (!interval.intersects(itr->second)) {
      // we only consider those live interval which doesn't interfere with current
      // interval.
      // No matching in register class should be ignored.
      unsigned reg = itr->second->reg;
      // Avoiding LiveIn register(such as argument register).
      if (tri->getMinimalPhysRegClass(reg) != &rc || mri->isLiveIn(reg))
        continue;

      //FIXME, we should check whether anti-dependence will occurs after inserting a move instr.
      return reg;
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
    auto mi = region->inst_begin();
    auto mie = region->inst_end();
    // Step#2: visits register operand of each machine instr in the program sequence.
    for (; mi != mie; ++mi) {
      // Step#3: collects reg definition information.
      // Step#4: collects reg uses information.
      collectRefDefUseInfo(*mi, region);
    }
  }

  // Step#5: checks if we should rename the defined register according to usd-def,
  //         If it is, construct a pair of use-def reg pair.
  for (AntiDepPair pair : antiDeps) {
    if (shouldRename(pair)) {
      // Step#6: choose a physical register for renaming.
      // Step#7: if there is no free physical register, using heuristic method to
      //         spill out a interval.
      auto useMI = pair.use->getParent();
      unsigned phyReg = choosePhysRegForRenaming(pair.use);
      assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
      assert(phyReg != pair.use->getReg());

      // Step#8: insert a move instruction before use of use-def pair.
      MachineBasicBlock *parent = useMI->getParent();
      tii->copyPhysReg(*parent, useMI, DebugLoc(), phyReg, pair.use->getReg(), false);
      MachineBasicBlock::iterator itr2(useMI);

      // Step#9: insert a splitting boundary(means Idem intrinsic call instr) after move instr
      ++itr2;
      // The new region starts at I.
      tii->emitIdemBoundary(*parent, itr2);
    }
  }
  return true;
}