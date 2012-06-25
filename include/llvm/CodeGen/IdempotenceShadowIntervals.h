//===-------- IdempotenceShadowIntervals.h ----------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the interface for querying the idempotence ``shadow''
// information for a given virtual register.  The shadow interval of a virtual
// register is the interval over which the storage resource allocated to the
// virtual register may not be overwritten by some other virtual register to
// preserve the idempotence property.
//
// A virtual register whose live interval does not cross any idempotence
// boundary (is not live at any boundary) will necessarily not have any shadow
// interval.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_IDEMPOTENCESHADOWINTERVALS_H
#define LLVM_CODEGEN_IDEMPOTENCESHADOWINTERVALS_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/IdempotenceOptions.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineIdempotentRegions.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"

#include "IdempotenceUtils.h"

#include <sstream>

namespace llvm {

class IdempotenceShadowIntervals;
class ShadowInterval {
 public:
  typedef SlotInterval::Allocator Allocator;

  ShadowInterval(SlotInterval::Allocator *Allocator,
                 const LiveInterval &LI,
                 const IdempotenceShadowIntervals &ISI)
    : Slots_(*Allocator), LI_(LI), ISI_(ISI) {}

  // This shadow's live interval or the region construction has changed.
  // Recompute this shadow.
  void recompute();

  // Return the size of this shadow measured in units of SlotIndexes.  Analogous
  // to LiveInterval::getSize().
  unsigned getSize() const;

  // Return the live interval associated with this shadow.
  const LiveInterval &getInterval() const { return LI_; }

  // Return whether this shadow is clobbered by the instruction MI.
  bool isClobberedByMI(const MachineInstr &MI) const;

  // Return whether this shadow is clobbered by any definitions of the live interval LI.
  bool isClobberedByLI(const LiveInterval &LI) const;

  // Return whether this shadow would be clobbered by a CSR restore of Reg.
  bool isClobberedByCalleeSavedRestoreOf(unsigned Reg) const;

  // Return whether this shadow is overlapped by the live interval LI.
  bool isOverlappedByLI(const LiveInterval &LI) const;

  // Return whether this shadow exists at the slot index Slot.
  bool isShadowAt(SlotIndex Slot) const {
    return Slots_.lookup(Slot);
  }

  // Debugging support.
  void print(raw_ostream &OS) const;

 private:
  friend class IdempotenceShadowIntervals;

  SlotInterval Slots_;
  const LiveInterval &LI_;
  const IdempotenceShadowIntervals &ISI_;

  // Do not implement.
  ShadowInterval();
};

raw_ostream &operator<<(raw_ostream &OS, const ShadowInterval &SI);

class IdempotenceShadowIntervals : public MachineFunctionPass {
 public:
  static char ID;
  IdempotenceShadowIntervals() : MachineFunctionPass(ID) {
    initializeIdempotenceShadowIntervalsPass(*PassRegistry::getPassRegistry());
  }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const;
  virtual void releaseMemory();
  virtual bool runOnMachineFunction(MachineFunction &MF);

  // Get the shadow interval corresponding with the live interval LI.
  ShadowInterval &getShadow(const LiveInterval &LI) {
    ShadowIntervalMap::const_iterator It = ShadowIntervalMap_.find(LI.reg);
    if (It == ShadowIntervalMap_.end())
      return *createShadow(LI);
    return *It->second;
  }
  const ShadowInterval &getShadow(const LiveInterval &LI) const {
    return const_cast<IdempotenceShadowIntervals *>(this)->getShadow(LI);
  }

  // Recompute the shadow interval corresponding with the live interval LI.
  // Equivalent to: (forgetShadow(LI), getShadow(LI)) but faster.
  ShadowInterval &recomputeShadow(const LiveInterval &LI) {
    ShadowIntervalMap::const_iterator It = ShadowIntervalMap_.find(LI.reg);
    if (It == ShadowIntervalMap_.end())
      return *createShadow(LI);
    computeShadow(It->second);
    return *It->second;
  }

  // Forget the shadow interval corresponding with the live interval LI.  
  // Note: this isn't called everywhere it should be but it's partly because
  // RegisterCoalescer is such a mess.  Not bothered.
  void forgetShadow(const LiveInterval &LI) {
    ShadowIntervalMap::iterator It = ShadowIntervalMap_.find(LI.reg);
    if (It == ShadowIntervalMap_.end())
      return;
    delete It->second;
    ShadowIntervalMap_.erase(It);
  }

  // Return whether register coalescing of SrcLI and DstLI is safe.
  bool isRegisterCoalescingSafe(const LiveInterval &SrcLI,
                                const LiveInterval &DstLI) const;

  // Return whether stack slot coalescing of SrcLI and DstLI is safe.
  bool isStackSlotCoalescingSafe(const LiveInterval &SrcLI,
                                 const LiveInterval &DstLI) const;

  // Verify that no region has a live-in register that is overwritten inside
  // the region.
  bool verify(const VirtRegMap *VRM = 0) const;

  // IgnoreQuery is an abstract class to support ignoring of machine
  // instructions in the construction and verification of shadow intervals.
  // Currently used to just ignore the joined copies (intermediate undefs)
  // produced while RegisterCoalescer runs.
  struct IgnoreQuery : std::unary_function<MachineInstr *, bool> {
    virtual bool operator () (MachineInstr *MI) = 0;
  };
  struct DefaultIgnoreQuery : IgnoreQuery {
    virtual bool operator () (MachineInstr *MI) { return false; }
  };
  class ScopedIgnoreQuerySetter {
   public:
    ScopedIgnoreQuerySetter(IdempotenceShadowIntervals *ISI, IgnoreQuery *Query)
        : ISI_(ISI) {
      if (ISI_)
        ISI_->IgnoreQuery_ = Query;
    }
    ~ScopedIgnoreQuerySetter() {
      if (ISI_)
        ISI_->IgnoreQuery_ = &ISI_->DefaultIgnoreQuery_;
    };
   private:
    IdempotenceShadowIntervals *ISI_;
  };

  // Shortcuts for setting up this analysis based on need.
  static IdempotenceShadowIntervals *getAnalysisForPreservation(const Pass &P) {
    if (IdempotencePreservationMode != IdempotenceOptions::NoPreservation)
      return &P.getAnalysis<IdempotenceShadowIntervals>();
    return NULL;
  }
  static void requireAnalysisForPreservation(AnalysisUsage *AU) {
    if (IdempotencePreservationMode != IdempotenceOptions::NoPreservation)
      AU->addRequired<IdempotenceShadowIntervals>();
  }

 private:
  friend class ShadowInterval;

  // ShadowIntervalMap provides ShadowInterval lookup by register number.
  typedef DenseMap<unsigned, ShadowInterval *> ShadowIntervalMap;

  // ShadowInterval allocator and map.
  mutable ShadowInterval::Allocator Allocator_;
  mutable ShadowIntervalMap ShadowIntervalMap_;

  // Cache function exit block information to compute callee-saved clobbers.
  typedef SmallVector<MachineBasicBlock *, 8> FunctionExitBlocks;
  FunctionExitBlocks FunctionExitBlocks_;

  // A query to check for instructions that should be ignored when computing
  // shadow intervals.
  IgnoreQuery *IgnoreQuery_;
  DefaultIgnoreQuery DefaultIgnoreQuery_;

  // The usual.
  MachineFunction          *MF_;
  MachineIdempotentRegions *MIR_;
  SlotIndexes              *SLI_;
  MachineRegisterInfo      *MRI_;
  const TargetRegisterInfo *TRI_;

  // Return whether MI should be ignored for the purposes of computing shadow
  // intervals and whether they are clobbered.
  bool shouldIgnore(const MachineInstr &MI) const {
    if ((*IgnoreQuery_)(const_cast<MachineInstr *>(&MI)))
      return true;
    
    // RegisterCoalescer may not yet have gotten to this one but other
    // coalescing has turned it into an identity copy.
    return (MI.isIdentityCopy() || MI.isKill());
  }

  // Create the shadow interval for the live interval LI.
  ShadowInterval *createShadow(const LiveInterval &LI) const {
    ShadowInterval *&SI = ShadowIntervalMap_[LI.reg];
    assert(!SI);
    SI = new ShadowInterval(&Allocator_, LI, *this);
    computeShadow(SI);
    return SI;
  }

  // Compute a shadow SI.
  void computeShadow(ShadowInterval *SI) const;
  void computeShadowForRegion(const IdempotentRegion &Region,
                              ShadowInterval *SI) const;
  void computeShadowForRange(const SlotIndex Start,
                             const SlotIndex End,
                             ShadowInterval *SI) const;

  // Helper for computeShadowForRegion().
  void mapRegionSlotsUpToDefsOfLI(const IdempotentRegion &Region,
                                  const LiveInterval &LI,
                                  SlotInterval *Slots) const;

  // Verify that the range [Start,End) does not clobber Reg.
  void verifyRange(const SlotIndex Start,
                   const SlotIndex End,
                   const unsigned Reg) const;
};

}
#endif



