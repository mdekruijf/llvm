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
    RegisterRenaming() : MachineFunctionPass(ID) {}
    virtual bool runOnMachineFunction(MachineFunction &MF);
    virtual void getAnalysisUsage(AnalysisUsage &AU) {
      AU.setPreservesCFG();
      AU.addRequired<MachineIdempotentRegions>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }
    const char *getPassName() const {
      return "Register Renaming for Idempotence pass";
    }

  private:
    void collectRefDefUseInfo(MachineInstr *mi, IdempotentRegion *region);
    bool shouldRename(AntiDepPair pair);
    unsigned choosePhysRegForRenaming(MachineOperand *use);
    // records all def registers by instr before current mi.
    std::map<MachineInstr*, std::set<int>> prevDefRegs;
    // records all use registers of current mi and previous mi.
    std::map<MachineInstr*, std::set<MachineOperand*>> prevUseRegs;
    std::vector<AntiDepPair> antiDeps;
    const TargetInstrInfo *tii;
    const TargetRegisterInfo *tri;
    const MachineFunction *mf;
  };
}


char RegisterRenaming::ID = 0;

INITIALIZE_PASS_BEGIN(RegisterRenaming, "reg-renaming",
    "Register Renaming for Idempotence", false, false)
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

static void getDefUses(MachineInstr *mi, std::set<int> *defs, std::set<MachineOperand *> *uses) {
  for (unsigned i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand *mo = &mi->getOperand(i);
    if (!mo->isReg() || !mo->getReg()) continue;
    unsigned reg = mo->getReg();
    assert(TargetRegisterInfo::isPhysicalRegister(reg));
    if (mo->isDef() && defs)
      defs->insert(reg);
    else if (mo->isUse() && uses)
      uses->insert(mo);
  }
}

void RegisterRenaming::collectRefDefUseInfo(MachineInstr *mi,
    IdempotentRegion *region) {
  std::set<MachineOperand*> uses;
  std::set<int> defs;
  getDefUses(mi, &defs, &uses);

  if (defs.empty() || defs.size() != 1)
    return;

  if (mi == &region->getEntry()) {
    prevDefRegs[mi] = std::set<int>();
    prevUseRegs[mi] = uses;
  }
  else {
    // if the mi is the first mi of basic block with preds.
    if (mi == &mi->getParent()->front()) {
      MachineBasicBlock *mbb = mi->getParent();
      if (mbb->pred_empty()) {
        prevDefRegs[mi] = std::set<int>();
        prevUseRegs[mi] = uses;
      }
      else {
        std::set<int> &predDefs = prevDefRegs[mi];
        std::set<MachineOperand*> &predUses = prevUseRegs[mi];

        auto itr = mbb->pred_begin();
        auto end = mbb->pred_end();
        for (; itr != end; ++itr) {
          MachineBasicBlock *pred = *itr;
          MachineInstr *predMI = &*pred->getLastNonDebugInstr();
          std::set<MachineOperand*> localUses;
          std::set<int> localDefs;
          getDefUses(predMI, &localDefs, &localUses);

          Union(predDefs, localDefs, prevDefRegs[predMI]);
          Union(predUses, localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    }
    // otherwise
    std::set<int> localPrevDefs;
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
  auto def = pair.def;
  MachineInstr *useMI = use->getParent();
  std::set<int> &defs = prevDefRegs[useMI];
  return !defs.count(use->getReg());
}

unsigned RegisterRenaming::choosePhysRegForRenaming(MachineOperand *use) {
  auto rc = tri->getMinimalPhysRegClass(use->getReg());
  auto allocSet = tri->getAllocatableSet(*mf, rc);
  // TODO
  return 0;
}

bool RegisterRenaming::runOnMachineFunction(MachineFunction &MF) {
  mf = &MF;
  tii = MF.getTarget().getInstrInfo();
  tri = MF.getTarget().getRegisterInfo();

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