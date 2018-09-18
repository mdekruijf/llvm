//
// Created by xlous on 9/12/18.
//

#include <deque>
#include <vector>
#include "IdemUtil.h"

using namespace std;
using namespace llvm;

void llvm::computeReversePostOrder(MachineFunction &MF,
                             MachineDominatorTree &dt,
                             std::vector<MachineBasicBlock *> &sequence) {
  unsigned size = MF.getNumBlockIDs();
  long *numIncomingBranches = new long[size];
  {
    unsigned idx = 0;
    for (MachineFunction::iterator itr = MF.begin(), end = MF.end();
         itr != end; ++itr) {
      long numPreds = std::distance(itr->pred_begin(), itr->pred_end());
      for (auto predItr = itr->pred_begin(), predEnd = itr->pred_end();
           predItr != predEnd; ++predItr) {
        if (dt.dominates(&*itr, *predItr))
          --numPreds;
      }
      numIncomingBranches[idx] = numPreds;
      ++idx;
    }
  }

  std::deque<MachineBasicBlock *> worklist;
  worklist.push_back(&MF.front());
  while (!worklist.empty()) {
    MachineBasicBlock *curMBB = worklist.front();
    worklist.pop_front();
    sequence.push_back(curMBB);

    for (auto itr = curMBB->succ_begin(), end = curMBB->succ_end(); itr != end; ++itr) {
      auto succ = *itr;
      --numIncomingBranches[succ->getNumber()];
      if (!numIncomingBranches[succ->getNumber()])
        worklist.push_back(succ);
    }
  }

  delete[] numIncomingBranches;
}

bool llvm::reachable(MachineInstr *A, MachineInstr *B) {
  if (!A || !B) return false;

  if (!A->getParent() || !B->getParent())
    return false;

  if (A->getParent() == B->getParent()) {
    for (MachineInstr &mi : *A->getParent()) {
      if (&mi == A)
        return true;
      else if (&mi == B)
        return false;
    }
    assert(false);
  }

  if (A->getParent() != B->getParent()) {
    MachineBasicBlock *MBBA = A->getParent();
    MachineBasicBlock *MBBB = B->getParent();

    std::vector<MachineBasicBlock*> worklist;
    worklist.push_back(MBBA);

    std::set<MachineBasicBlock*> visited;

    while (!worklist.empty()) {
      auto cur = worklist.back();
      worklist.pop_back();
      if (!visited.count(cur))
        continue;

      if (cur == MBBB)
        return true;

      std::for_each(cur->succ_begin(), cur->succ_end(), [&](MachineBasicBlock *mbb)
            { return worklist.push_back(mbb); });
    }
  }
  return false;
}

char llvm::IdemInstrScavenger::ID = 0;

