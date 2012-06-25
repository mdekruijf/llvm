//===-------- ConstructIdempotentRegions.cpp --------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This transformation pass is just a consumer of MemoryIdempotenceAnalysis.  It
// inserts the actual idempotence boundary instructions as intrinsics into the
// LLVM IR.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "construct-idempotent-regions"
#include "llvm/Function.h"
#include "llvm/Instruction.h"
#include "llvm/Instructions.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/Module.h"
#include "llvm/CodeGen/IdempotenceOptions.h"
#include "llvm/CodeGen/MemoryIdempotenceAnalysis.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include <vector>
using namespace llvm;

class ConstructIdempotentRegions : public FunctionPass {
public:
  static char ID;  // Pass identification, replacement for typeid
  ConstructIdempotentRegions() : FunctionPass(ID) {
    initializeConstructIdempotentRegionsPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnFunction(Function &F);
  virtual void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.addRequired<MemoryIdempotenceAnalysis>();
    AU.setPreservesAll();
  }
};

char ConstructIdempotentRegions::ID = 0;
INITIALIZE_PASS_BEGIN(ConstructIdempotentRegions,
    "construct-idempotent-regions",
    "Idempotent Region Construction", "true", "false")
INITIALIZE_PASS_DEPENDENCY(MemoryIdempotenceAnalysis)
INITIALIZE_PASS_END(ConstructIdempotentRegions,
    "construct-idempotent-regions",
    "Idempotent Region Construction", "true", "false")

FunctionPass *llvm::createConstructIdempotentRegionsPass() {
  return new ConstructIdempotentRegions();
}

bool ConstructIdempotentRegions::runOnFunction(Function &F) {
  assert(IdempotenceConstructionMode != IdempotenceOptions::NoConstruction &&
         "pass should not be run");

  // Iterate over the analysis cut points and insert cuts.
  MemoryIdempotenceAnalysis *MIA = &getAnalysis<MemoryIdempotenceAnalysis>();
  for (MemoryIdempotenceAnalysis::const_iterator I = MIA->begin(),
        E = MIA->end(); I != E; ++I) {
    Function *Idem = Intrinsic::getDeclaration(F.getParent(), Intrinsic::idem);
    CallInst::Create(Idem, "", *I);
  }
  return true;
}


