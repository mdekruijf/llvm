//===-------- IdempotenceShadowIntervals.cpp --------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the implementation for querying the idempotence ``shadow''
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

#define DEBUG_TYPE "idempotence-intervals"
#include "llvm/CodeGen/IdempotenceShadowIntervals.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/IdempotenceOptions.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineIdempotentRegions.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/PassSupport.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetRegisterInfo.h"

#include "IdempotenceUtils.h"
#include "RegisterCoalescer.h"
#include "VirtRegMap.h"

#include <algorithm>
#include <sstream>

using namespace llvm;

//===----------------------------------------------------------------------===//
// Helpers
//===----------------------------------------------------------------------===//

// Return the first sub-range inside [CandidateStart, CandidateEnd] that does
// not conflict with [ConflictingStart, ConflictingEnd] or any other range
// potentially existing after ConflictingEnd.  Assumes ConflictingEnd >
// CandidateStart; hence, overlap potential.
//
// Returns uninitialized SlotIndexes if there is no overlapping sub-range.
// Returns in NextCandidateStart the start point of the next potentially
// non-overlapping sub-range after ConflictingEnd.  
static std::pair<SlotIndex, SlotIndex> getNonOverlappingSubRange(
    const SlotIndex CandidateStart,
    const SlotIndex CandidateEnd,
    const SlotIndex ConflictingStart,
    const SlotIndex ConflictingEnd,
    SlotIndex *NextCandidateStart) {
  assert(CandidateStart < CandidateEnd &&
         ConflictingStart < ConflictingEnd && "malformed range");
  assert(ConflictingEnd >= CandidateStart && "invariant does not hold");

  // Check for no overlap, i.e. the entire candidate range is non-overlapping.
  if (ConflictingStart >= CandidateEnd) {
    *NextCandidateStart = CandidateEnd;
    return std::make_pair(CandidateStart, CandidateEnd);
  }

  // There is some overlap.  In all cases, any next non-overlapping range comes
  // on or after ConflictingEnd.
  *NextCandidateStart = ConflictingEnd;

  // Check for partial overlap over CandidateStart. In this case, no
  // non-overlapping range exists before ConflictingEnd.
  if (ConflictingStart <= CandidateStart)
    return std::make_pair(SlotIndex(), SlotIndex());

  // Must be partial overlap after CandidateStart.  Non-overlapping range
  // exists up to ConflictingStart. 
  return std::make_pair(CandidateStart, ConflictingStart);
}

//===----------------------------------------------------------------------===//
// ShadowInterval
//===----------------------------------------------------------------------===//

void ShadowInterval::recompute() {
  ISI_.computeShadow(this);
}

unsigned ShadowInterval::getSize() const {
  unsigned Sum = 0;
  for (SlotInterval::const_iterator I = Slots_.begin(); I.valid(); ++I)
    Sum += I.start().distance(I.stop());
  return Sum;
}

bool ShadowInterval::isClobberedByMI(const MachineInstr &MI) const {
  // Ignore copies from this shadow interval's live interval. These do not
  // clobber because the copied value already exists in the shadow.
  if (MI.isCopy()) {
    assert(MI.getOperand(0).getReg() != LI_.reg && "unexpected");
    if (MI.getOperand(1).getReg() == LI_.reg)
      return false;
  }

  // Calls don't clobber anything.  Their implicit def clobbers are protected by
  // an idempotence boundary at the entry of the called function.
  if (MI.isCall())
    return false;

  // Some things we may have been asked to ignore.
  if (ISI_.shouldIgnore(MI))
    return false;

  return Slots_.lookup(ISI_.SLI_->getInstructionIndex(&MI).getRegSlot());
}

bool ShadowInterval::isClobberedByLI(const LiveInterval &LI) const {
  for (MachineRegisterInfo::def_iterator D = ISI_.MRI_->def_begin(LI.reg),
       DE = ISI_.MRI_->def_end(); D != DE; ++D)
    if (isClobberedByMI(*D))
      return true;
  return false;
}

bool ShadowInterval::isClobberedByCalleeSavedRestoreOf(unsigned Reg) const {
  assert(TargetRegisterInfo::isPhysicalRegister(Reg));
  if (isCalleeSavedRegister(Reg, *(ISI_.TRI_))) {
    for (IdempotenceShadowIntervals::FunctionExitBlocks::const_iterator
         I = ISI_.FunctionExitBlocks_.begin(),
         E = ISI_.FunctionExitBlocks_.end(); I != E; ++I) {
      SlotIndex Proxy = ISI_.SLI_->getInstructionIndex(&(*I)->back());
      if (Slots_.lookup(Proxy.getBaseIndex())) {
        DEBUG(dbgs() << "Clobbered by CSR " << PrintReg(Reg, ISI_.TRI_) << "\n");
        return true;
      }
    }
  }
  return false;
}

bool ShadowInterval::isOverlappedByLI(const LiveInterval &LI) const {
  for (SlotInterval::const_iterator I = Slots_.begin(); I.valid(); ++I) {
    LiveInterval::const_iterator J = LI.find(I.start());
    if (J == LI.end())
      return false;

    assert(J->end > I.start() && "LI find() invariant broken");
    if (J->start < I.stop())
      return true;
  }
  return false;
}

void ShadowInterval::print(raw_ostream &OS) const {
  OS << "ShadowInterval " << PrintReg(LI_.reg, ISI_.TRI_) << " = ";
  if (Slots_.empty()) {
    OS << "empty";
    return;
  }
  for (SlotInterval::const_iterator I = Slots_.begin(); I.valid(); ++I)
    OS << "[" << I.start() << ',' << I.stop() << ")";
}

raw_ostream &llvm::operator<<(raw_ostream &OS, const ShadowInterval &SI) {
  SI.print(OS);
  return OS;
}

//===----------------------------------------------------------------------===//
// IdempotenceShadowIntervals
//===----------------------------------------------------------------------===//

char IdempotenceShadowIntervals::ID = 0;

INITIALIZE_PASS_BEGIN(IdempotenceShadowIntervals,
                      "idempotence-shadow-intervals",
                      "Idempotence Shadow Interval Analysis", false, true)
INITIALIZE_PASS_DEPENDENCY(SlotIndexes)
INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
INITIALIZE_PASS_END(IdempotenceShadowIntervals,
                    "idempotence-shadow-intervals",
                    "Idempotence Shadow Interval Analysis", false, true)

void IdempotenceShadowIntervals::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequiredTransitive<SlotIndexes>();
  AU.addRequiredTransitive<MachineIdempotentRegions>();
  AU.setPreservesAll();
  MachineFunctionPass::getAnalysisUsage(AU);
}

void IdempotenceShadowIntervals::releaseMemory() {
  for (ShadowIntervalMap::iterator I = ShadowIntervalMap_.begin(),
       IE = ShadowIntervalMap_.end(); I != IE; ++I) {
    ShadowInterval *SI = I->second;
    assert(SI);
    delete SI;
  }
  ShadowIntervalMap_.clear();
  FunctionExitBlocks_.clear();
}

bool IdempotenceShadowIntervals::runOnMachineFunction(MachineFunction &MF) {
  assert(IdempotenceConstructionMode != IdempotenceOptions::NoConstruction &&
         (IdempotencePreservationMode != IdempotenceOptions::NoPreservation ||
          IdempotenceConstructionMode == IdempotenceOptions::OptimizeForSpeed)
          && "pass should not be run");

  MF_  = &MF;
  MIR_ = &getAnalysis<MachineIdempotentRegions>();
  SLI_ = &getAnalysis<SlotIndexes>();
  MRI_ = &MF_->getRegInfo();
  TRI_ = MF_->getTarget().getRegisterInfo();
  IgnoreQuery_ = &DefaultIgnoreQuery_;

  // Cache exit block information for checking callee-saved register clobbers.
  for (MachineFunction::iterator BB = MF_->begin(); BB != MF_->end(); ++BB)
    if (!BB->empty() && BB->succ_empty())
      FunctionExitBlocks_.push_back(BB);

  // Nothing else to do.  Compute on demand.
  return false;
}

void IdempotenceShadowIntervals::computeShadow(ShadowInterval *SI) const {
  SI->Slots_.clear();

  // Compute the shadow over each region in turn.
  // FIXME: This algorithm could be faster since there is no caching of region
  // information and other optimizations may be possible.  It is also called too
  // often since I haven't thought about a good way to perform just the minimal
  // amount of re-computation.  Both are not worth my time right now.
  DEBUG(dbgs() << "\tComputing idempotence shadow for "
        << SI->getInterval() << "\n");
  for (MachineIdempotentRegions::const_iterator R = MIR_->begin(),
       RE = MIR_->end(); R != RE; ++R) 
    computeShadowForRegion(**R, SI);

  DEBUG(dbgs() << "\t\tproduced " << *SI << "\n");
}

void IdempotenceShadowIntervals::mapRegionSlotsUpToDefsOfLI(
    const IdempotentRegion &Region,
    const LiveInterval &LI,
    SlotInterval *Slots) const {

  Slots->clear();
  IdempotentRegion::const_mbb_iterator RI(Region);
  for (; RI.isValid(); ++RI) {
    SlotIndex Start, End;
    tie(Start, End) = RI.getSlotRange(*SLI_);

    // Walk the slot range to any def and update the iterator to skip successors
    // along the current depth-first search path.
    for (SlotIndex I = Start; I < End; I = I.getNextIndex()) {
      assert(I.isValid());
      MachineInstr *MI = SLI_->getInstructionFromIndex(I);
      if (MI && !shouldIgnore(*MI) && MI->definesRegister(LI.reg)) {
        RI.skip();
        End = I;
      }
    }
    Slots->insert(Start, End, true);
  }
}

void IdempotenceShadowIntervals::computeShadowForRegion(
    const IdempotentRegion &Region,
    ShadowInterval *SI) const {
  const LiveInterval *LI = &SI->getInterval();

  // If LI is not live-in to the region then there is no shadow.
  const MachineInstr *Entry = &Region.getEntry();
  SlotIndex EntrySlot = SLI_->getInstructionIndex(Entry).getRegSlot();
  if (!LI->liveAt(EntrySlot))
    return;

  // The shadow for a given (Region, LI) pairing depends on whether idempotence
  // assumes variable or invariable control flow on re-execution: 
  //
  // Case 1 -- invariable control: 
  //   A shadow stems from all uses of LI inside Region.  The shadow prevents
  //   the emergence of clobber antidependences; LI is trivially dead upon
  //   re-execution along those paths that do not follow from uses of LI and
  //   that are not already contained in LI.
  //
  // Case 2 -- variable control: 
  //   A shadow stems from the entry point of Region.  LI cannot be overwritten
  //   *anywhere* in the region because the value may be live down the correct
  //   path (statically unknown) on re-execution.
  DEBUG(dbgs() << "\t\tprocessing region "; Region.print(dbgs(), SLI_);
        dbgs() << "\n");

  // Compute the stem points.
  bool VerifyRanges = true;
  typedef SmallVector<const MachineInstr *, 4> StemMIsTy;
  StemMIsTy StemMIs;
  if (IdempotencePreservationMode == IdempotenceOptions::VariableCF) {
    StemMIs.push_back(Entry);
  } else if (TargetRegisterInfo::isStackSlot(LI->reg)) {
    // Stack slot registers don't have a use iterator that we can use, so we use
    // the entry instruction as a proxy.  It is pessimistic, but that's probably
    // okay.  Unfortunately, this will break verifyRange() for InvariableCF even
    // though there is actually no problem.  Can't be bothered to fix right now.
    StemMIs.push_back(Entry);
    VerifyRanges = (IdempotencePreservationMode ==
                    IdempotenceOptions::VariableCF);
  } else {
    // First compute the bounds of the region as a SlotInterval that we can
    // query to determine those uses of LI that fall inside the region.
    // Only compute bounds up to a def of LI; uses after defs of LI may be
    // "protected" by the def in the case of invariable control flow.
    SlotInterval RegionSlotsUpToDefs(Allocator_);
    mapRegionSlotsUpToDefsOfLI(Region, *LI, &RegionSlotsUpToDefs);

    // Now compute the uses not preceded by a def.
    for (MachineRegisterInfo::use_nodbg_iterator
         U = MRI_->use_nodbg_begin(LI->reg), UE = MRI_->use_nodbg_end();
         U != UE; ++U) {
      const MachineInstr *UseMI = &*U;
      SlotIndex UseSlot = SLI_->getInstructionIndex(UseMI).getRegSlot();
      if (!shouldIgnore(*UseMI) && RegionSlotsUpToDefs.lookup(UseSlot))
        StemMIs.push_back(UseMI);
    }
  }

  // For each instruction in StemMIs, scan forward to Region's exits and
  // compute shadows for the scanned ranges.
  SlotInterval StemSuccSlots(Allocator_);
  for (StemMIsTy::iterator S = StemMIs.begin(), SE = StemMIs.end();
       S != SE; ++S) {
    DEBUG(dbgs() << "\t\tanalyzing from stem @"
          << SLI_->getInstructionIndex(*S).getRegSlot() << "\n");
    mapSuccessorSlotsOfMIInRegion(**S, Region, *SLI_, &StemSuccSlots);
    for (SlotInterval::const_iterator I = StemSuccSlots.begin(),
         IE = StemSuccSlots.end(); I != IE; ++I) {
      if (IdempotenceVerify && VerifyRanges)
        verifyRange(I.start(), I.stop(), LI->reg);
      computeShadowForRange(I.start(), I.stop(), SI);
    }
  }
}

// computeShadowForRange - Compute the shadow given a candidate range [Start,
// End).  We need to remove sub-ranges already contained in SI since IntervalMap
// does not allow overlapping inserts.  Also remove the sub-ranges contained in
// LI to keep the shadow interval (SI) and the live interval (LI) disjoint.
void IdempotenceShadowIntervals::computeShadowForRange(
    const SlotIndex Start,
    const SlotIndex End,
    ShadowInterval *SI) const {
  const LiveInterval *LI = &SI->getInterval();

  // First find the sub-ranges where LI is not live.
  SlotIndex OuterStart = Start;
  SlotIndex OuterEnd = End;
  while (OuterStart < OuterEnd) {
    // Set defaults.
    SlotIndex InnerStart = OuterStart;
    SlotIndex InnerEnd = OuterEnd;

    // Query LI for any next potentially overlapping sub-range.
    LiveInterval::const_iterator OuterIt = LI->find(OuterStart);
    if (OuterIt != LI->end()) {
      assert(OuterStart < OuterIt->end && "LI find() invariant broken");
      tie(InnerStart, InnerEnd) = getNonOverlappingSubRange(
          OuterStart, OuterEnd, OuterIt->start, OuterIt->end, &OuterStart);
      if (!InnerStart.isValid())
        continue;
    }

    // Now find the sub-ranges of this sub-range not already contained in SI.
    while (InnerStart < InnerEnd) {
      // Set defaults.
      SlotIndex OKStart = InnerStart;
      SlotIndex OKEnd = InnerEnd;

      // Query SI for any next potentially overlapping sub-range.
      SlotInterval::const_iterator InnerIt = SI->Slots_.find(InnerStart);
      if (InnerIt != SI->Slots_.end()) {
        assert(InnerStart < InnerIt.stop() && "SI find() invariant broken");
        tie(OKStart, OKEnd) = getNonOverlappingSubRange(
            InnerStart, InnerEnd, InnerIt.start(), InnerIt.stop(), &InnerStart);
        if (!OKStart.isValid())
          continue;
      }

      // Now finally we can insert.
      SI->Slots_.insert(OKStart, OKEnd, true);
    }

    // Restart from the farthest point where overlap is unknown.
    OuterStart = std::max(InnerStart, OuterStart);
  }
}

static void dumpDidClobber(const ShadowInterval &Shadow,
                           const LiveInterval &LI,
                           const TargetRegisterInfo &TRI,
                           bool DidClobber) {
  dbgs() << "\t" << Shadow << "\n\tclobbered by " << LI << "?";
  if (DidClobber)
    dbgs() << " YES";
  else
    dbgs() << " NO";
  dbgs() << "\n";
}

bool IdempotenceShadowIntervals::isRegisterCoalescingSafe(
    const LiveInterval &SrcLI,
    const LiveInterval &DstLI) const {
  assert(!TargetRegisterInfo::isStackSlot(SrcLI.reg) &&
         !TargetRegisterInfo::isStackSlot(DstLI.reg) && "not registers");

  const ShadowInterval *SrcShadow = &getShadow(SrcLI);
  bool SrcClobber = SrcShadow->isClobberedByLI(DstLI);
  DEBUG(dumpDidClobber(*SrcShadow, DstLI, *TRI_, SrcClobber));

  const ShadowInterval *DstShadow = &getShadow(DstLI);
  bool DstClobber = DstShadow->isClobberedByLI(SrcLI);
  DEBUG(dumpDidClobber(*DstShadow, SrcLI, *TRI_, DstClobber));
  
  return !SrcClobber && !DstClobber;
}

bool IdempotenceShadowIntervals::isStackSlotCoalescingSafe(
    const LiveInterval &SrcLI,
    const LiveInterval &DstLI) const {
  assert((TargetRegisterInfo::isStackSlot(SrcLI.reg) ||
          TargetRegisterInfo::isStackSlot(DstLI.reg)) &&
         "at least one should be a stack slot");

  // Stack slot intervals have the really annoying property that both (1) their
  // VNInfo's do not define the def points and (2) defusechain_iterator doesn't
  // work on them.  Check for overlap instead of clobbers (pessimistic).
  return (!getShadow(SrcLI).isOverlappedByLI(DstLI) &&
          !getShadow(DstLI).isOverlappedByLI(SrcLI));
}

void IdempotenceShadowIntervals::verifyRange(const SlotIndex Start,
                                             const SlotIndex End,
                                             const unsigned Reg) const {
  bool Verified = true;
  DenseSet<unsigned> LiveIns;
  LiveIns.insert(Reg);
  for (SlotIndex I = Start; I < End; I = I.getNextIndex()) {
    assert(I.isValid());
    MachineInstr *MI = SLI_->getInstructionFromIndex(I);
    if (!MI || shouldIgnore(*MI))
      continue;
    Verified &= MIR_->verifyInstruction(*MI, LiveIns, SLI_);
  }

  if (!Verified) {
    errs() << "verifyRange failed for range [" << Start << ", " << End << ")\n";
    MF_->print(errs(), SLI_);
    assert(0 && "shadow range pre-verification failed");
  }
}

