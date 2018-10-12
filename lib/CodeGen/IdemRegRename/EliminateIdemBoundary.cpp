//===----- EliminateIdemBoundary.cpp - Eliminate all idem boundary insts --===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/PassSupport.h"

using namespace llvm;

namespace {

/// EliminateIdemBoundary This class designed for eliminating
/// all idempotence boundary instruction for evaluating performance
/// without considering the cost of boundary when compared with
/// wics's algorithm[PLDI'12]
///
/// @author Jianping Zeng
class EliminateIdemBoundary : public MachineFunctionPass {
public:
  static char ID;
  EliminateIdemBoundary() : MachineFunctionPass(ID) {
    initializeEliminateIdemBoundaryPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnMachineFunction(MachineFunction &MF) override;
  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
  const char *getPassName() const {
    return "Eliminate Idempotence boundary pass";
  }

};
}

char EliminateIdemBoundary::ID = 0;
INITIALIZE_PASS_BEGIN(EliminateIdemBoundary, "eliminate-idem-boundary",
                      "Eliminate Idempotence boundary pass", false, false)
INITIALIZE_PASS_END(EliminateIdemBoundary, "eliminate-idem-boundary",
                    "Eliminate Idempotence boundary pass", false, false)

bool EliminateIdemBoundary::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo *tii = MF.getTarget().getInstrInfo();

  auto mbbItr = MF.begin(), mbbEnd = MF.end();
  // We don't cope with empty function.
  if (mbbItr == mbbEnd) return false;

  std::vector<MachineInstr*> toDelete;

  for (; mbbItr != mbbEnd; ++mbbItr) {
    MachineBasicBlock *mbb = mbbItr;
    if (!mbb || mbb->empty())
      continue;

    for (auto mi = mbb->instr_begin(), end = mbb->instr_end(); mi != end; ++mi) {
      if (tii->isIdemBoundary(mi)) {
        toDelete.push_back(mi);
      }
    }
  }

  bool changed = false;
  if (!toDelete.empty()) {
    changed = true;
    for (auto mi : toDelete)
      mi->eraseFromParent();
  }

#ifndef NDEBUG
  {
    auto mbbItr = MF.begin(), mbbEnd = MF.end();
    // We don't cope with empty function.
    if (mbbItr == mbbEnd) return false;

    for (; mbbItr != mbbEnd; ++mbbItr) {
      MachineBasicBlock *mbb = mbbItr;
      if (!mbb || mbb->empty())
        continue;

      for (auto mi = mbb->instr_begin(), end = mbb->instr_end(); mi != end; ++mi)
        assert(!tii->isIdemBoundary(mi) && "Failed to eliminate idem boundary?");
    }
  }
#endif

  return changed;
}

FunctionPass *llvm::createEliminateIdemBoundaryPass() {
  return new EliminateIdemBoundary();
}


