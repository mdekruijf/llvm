//===----- IdemStatistic.cpp - Static statistic for idempotence ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/PassSupport.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;

namespace {
struct IdemStatistic : public MachineFunctionPass {
  static char ID;
  IdemStatistic() : MachineFunctionPass(ID) {
    initializeIdemStatisticPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnMachineFunction(MachineFunction &MF) override;

  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    AU.addRequired<MachineIdempotentRegions>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
  const char *getPassName() const {
    return "Statistic the static data for idempotent region";
  }
};
}

char IdemStatistic::ID = 0;

INITIALIZE_PASS_BEGIN(IdemStatistic, "idem-statistic",
    "Statistic the static data for idempotent region", false, false)
  INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
INITIALIZE_PASS_END(IdemStatistic, "idem-statistic",
    "Statistic the static data for idempotent region", false, false)


FunctionPass *llvm::createIdemStatisticPass() {
  return new IdemStatistic();
}

bool IdemStatistic::runOnMachineFunction(MachineFunction &MF) {
  MachineIdempotentRegions *MIR = getAnalysisIfAvailable<MachineIdempotentRegions>();
  assert(MIR && "No MachineIdempotentRegions pass found?");

  long num = std::distance(MIR->begin(), MIR->end());
  if (num > 0) {
    // records the number of instrs within each region.
    long *regionSizes = new long[num]{0};

    int i = 0;
    for (auto itr = MIR->begin(), end = MIR->end(); itr != end; ++itr) {
      regionSizes[i++] = std::distance((*itr)->inst_begin(), (*itr)->inst_end());
    }
    assert(i == num && "Number of regions does't matches!");

    llvm::errs()<<"********** Static statistic data for idempotent regions **********\n";
    llvm::errs()<<"The number of regions are: "<<num<<"\n";
    long totalSize = 0;
    for (i = 0; i < num; i++) {
      totalSize += regionSizes[i];
      llvm::errs() << "The number of instrs within region #" << i << " are: " << regionSizes[i] << "\n";
    }
    long averageSize = totalSize/num;
    llvm::errs()<<"The average size of regions is: "<<averageSize<<"\n";
    llvm::errs()<<"\n";
  }
  return false;
}