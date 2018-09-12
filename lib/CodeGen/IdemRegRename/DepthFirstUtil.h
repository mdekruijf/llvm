#ifndef LLVM_DEPTHFIRSTUTIL_H
#define LLVM_DEPTHFIRSTUTIL_H

#include <llvm/CodeGen/MachineFunction.h>
#include <llvm/CodeGen/MachineDominators.h>
#include "llvm/CodeGen/MachineBasicBlock.h"

namespace llvm {

void computeReversePostOrder(MachineFunction &MF,
    MachineDominatorTree &dt,
    std::vector<MachineBasicBlock *> &sequence);

}

#endif //LLVM_DEPTHFIRSTUTIL_H
