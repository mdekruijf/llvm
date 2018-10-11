//===----- IdemStatistic.cpp - Static statistic for idempotence ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Path.h>
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/PassSupport.h"
#include "llvm/CodeGen/Passes.h"
#include <cstring>
#include <libgen.h>
#include <sys/stat.h>

using namespace llvm;
using namespace llvm::sys::fs;

namespace {
struct IdemStatistic : public MachineFunctionPass {
  static char ID;
  IdemStatistic() : MachineFunctionPass(ID) {
    initializeIdemStatisticPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnMachineFunction(MachineFunction &MF) override;

  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    /*AU.addRequired<MachineIdempotentRegions>();
    AU.addPreserved<MachineIdempotentRegions>();*/
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
//  INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
INITIALIZE_PASS_END(IdemStatistic, "idem-statistic",
    "Statistic the static data for idempotent region", false, false)


FunctionPass *llvm::createIdemStatisticPass() {
  return new IdemStatistic();
}

bool IdemStatistic::runOnMachineFunction(MachineFunction &MF) {
  MachineIdempotentRegions *MIR = (MachineIdempotentRegions*)llvm::createMachineIdempotentRegionsPass();
  assert(MIR && "No MachineIdempotentRegions pass found?");

  // Use this method to avoid a Pass Required assertion on Stack Coloring pass.
  // I don't want to solve it at all as yet!!!!
  // Jianping Zeng.
  MIR->runOnMachineFunction(MF);
  auto mfName = MF.getFunction()->getName();
  long num = MIR->size();

  char* path = const_cast<char*>(llvm::IdemStatisticOutFile.c_str());
  llvm::sys::Path p(path);
  std::string error;
  auto dir = llvm::sys::Path(p.getDirname());
  if (!dir.exists()) {
    if (mkdir(dir.c_str(), 0777)) {
      llvm::errs() << "can not create '" <<dir.c_str()<< "'\n";
      exit(-1);
    }
  }

  llvm::raw_fd_ostream os(path, error, llvm::raw_fd_ostream::F_Append);
  if (!error.empty()) {
    llvm::errs()<<error<<"\n";
    exit(-1);
  }

  os<<"\n********** Idem Static statistic data for function '"<<mfName<<"'' **********\n";
  os<<"The number of regions are: "<<num<<"\n";

  if (num > 0) {
    // records the number of instrs within each region.
    long *regionSizes = new long[num]{0};

    int i = 0;
    for (auto &itr : *MIR) {
      regionSizes[i++] = std::distance(itr->inst_begin(), itr->inst_end());
    }
    assert(i == num && "Number of regions does't matches!");

    long totalSize = 0;
    for (i = 0; i < num; i++) {
      totalSize += regionSizes[i];
      os << "The number of instrs within region #" << i << " are: " << regionSizes[i] << "\n";
    }
    long averageSize = totalSize/num;
    os<<"The average size of regions is: "<<averageSize<<"\n";
    os<<"\n";
  }
  os.close();

  return false;
}