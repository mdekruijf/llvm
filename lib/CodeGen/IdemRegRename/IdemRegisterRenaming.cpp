//===----- IdemRegisterRenaming.cpp - Register regnaming after RA ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <llvm/PassSupport.h>
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"

#define DEBUG_TYPE "reg-renaming"

using namespace llvm;

/// @author Jianping Zeng.
namespace {
  class RegisterRenaming : public MachineFunctionPass {
  public:
    static char ID;
    RegisterRenaming() : MachineFunctionPass(ID) {}
    virtual bool runOnMachineFunction(MachineFunction &MF);
    virtual void getAnalysisUsage(AnalysisUsage &AU) {
      AU.setPreservesCFG();
      MachineFunctionPass::getAnalysisUsage(AU);
    }
    const char *getPassName() const {
      return "Register Renaming for Idempotence pass";
    }
  };
}


char RegisterRenaming::ID = 0;

INITIALIZE_PASS_BEGIN(RegisterRenaming, "reg-renaming",
    "Register Renaming for Idempotence", false, false)
INITIALIZE_PASS_END(RegisterRenaming, "reg-renaming",
                    "Register Renaming for Idempotence", false, false)


FunctionPass* llvm::createRegisterRenamingPass() {
  return new RegisterRenaming();
}

//=== Implementation for class RegisterRenaming.  ====//

bool RegisterRenaming::runOnMachineFunction(MachineFunction &MF) {
  // Step#1: Collects regions
  // Step#2: visits register operand of each machine instr in the program sequence.
  // Step#3: collects reg definition information.
  // Step#4: collects reg uses information.
  // Step#5: checks if we should rename the defined register according to usd-def,
  //         If it is, construct a pair of use-def reg pair.
  // Step#6: choose a physical register for renaming.
  // Step#7: if there is no free physical register, using heuristic method to
  //         spill out a interval.
  // Step#8: insert a move instruction before use of use-def pair.
  // Step#9: insert a splitting boundary(means Idem intrinsic call instr) after move instr
  return true;
}