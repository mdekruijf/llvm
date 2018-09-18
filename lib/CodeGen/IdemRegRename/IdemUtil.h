#ifndef LLVM_DEPTHFIRSTUTIL_H
#define LLVM_DEPTHFIRSTUTIL_H

#include <llvm/CodeGen/MachineFunction.h>
#include <llvm/CodeGen/MachineDominators.h>
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

void computeReversePostOrder(MachineFunction &MF,
                             MachineDominatorTree &dt,
                             std::vector<MachineBasicBlock *> &sequence);
/**
 * Checks if it is reachable from MI A to B. Return true if it reaches.
 * @param A
 * @param B
 * @return
 */
bool reachable(MachineInstr *A, MachineInstr *B);

/**
 * Used for cleaning some redundant idem call instruction after register renaming.
 *
 * Virginia polytechnic and state University.
 * @author Jianping Zeng
 */
class IdemInstrScavenger : public MachineFunctionPass {
public:
  static char ID;

  IdemInstrScavenger() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override;

  virtual const char* getPassName() const override {
    return "Idempotence scavenger after register renaming";
  };

  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

}
#endif //LLVM_DEPTHFIRSTUTIL_H
