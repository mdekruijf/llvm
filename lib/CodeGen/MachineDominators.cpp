//===- MachineDominators.cpp - Machine Dominator Calculation --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements simple dominator construction algorithms for finding
// forward dominators on machine functions.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/Passes.h"

using namespace llvm;

namespace llvm {
TEMPLATE_INSTANTIATION(class DomTreeNodeBase<MachineBasicBlock>);
TEMPLATE_INSTANTIATION(class DominatorTreeBase<MachineBasicBlock>);
}

char MachineDominatorTree::ID = 0;

INITIALIZE_PASS(MachineDominatorTree, "machinedomtree",
                "MachineDominator Tree Construction", true, true)

char &llvm::MachineDominatorsID = MachineDominatorTree::ID;

void MachineDominatorTree::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();
  MachineFunctionPass::getAnalysisUsage(AU);
}

bool MachineDominatorTree::runOnMachineFunction(MachineFunction &F) {
  DT->recalculate(F);

  return false;
}

MachineDominatorTree::MachineDominatorTree()
    : MachineFunctionPass(ID) {
  initializeMachineDominatorTreePass(*PassRegistry::getPassRegistry());
  DT = new DominatorTreeBase<MachineBasicBlock>(false);
}

MachineDominatorTree::~MachineDominatorTree() {
  delete DT;
}

void MachineDominatorTree::releaseMemory() {
  DT->releaseMemory();
}

void MachineDominatorTree::print(raw_ostream &OS, const Module*) const {
  DT->print(OS);
}

// dominates - Return true if A dominates a use in B. This performs the
// special checks necessary if A and B are in the same basic block.
bool MachineDominatorTree::dominates(
    const MachineInstr *A, const MachineInstr *B) const {
  const MachineBasicBlock *BBA = A->getParent();
  const MachineBasicBlock *BBB = B->getParent();
  
  if (BBA != BBB) return dominates(BBA, BBB);
  
  // It is not possible to determine dominance between two PHI nodes 
  // based on their ordering.
  if (A->isPHI() && B->isPHI()) 
    return false;
  
  // Loop through the basic block until we find A or B.
  MachineBasicBlock::const_iterator I = BBA->begin();
  for (; &*I != A && &*I != B; ++I)
    /*empty*/;
  
  return &*I == A;
}
