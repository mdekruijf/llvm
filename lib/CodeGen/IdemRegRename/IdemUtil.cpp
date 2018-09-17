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
}

char llvm::IdemInstrScavenger::ID = 0;

