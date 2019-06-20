//===-------- DivideMachineIdempotentRegions.cpp ----------------*- C++ -*-===//
//
// Fundamentally, our speed optimization goals with idempotent region
// construction are to minimize register pressure by (1) minimizing the number
// of region boundary crossings, to amortize the cost to preserve live state at
// each boundary crossing, and (2) minimizing the amount of live register state
// at idempotent region boundary crossings.
//
// Goals (1) and (2) can be in conflict with one another.  Goal (1) is handled
// approximately by the IR-level MemoryIdempotenceAnalysis by attempting to
// cut as many antidependences as possible at once, and also by placing
// boundaries at antidependent stores rather than dominating stores, the latter
// of which, by definition, execute more frequently.  However, this placement
// may be sub-optimal in terms of minimizing the number of registers that are
// kept live across boundaries to maximize performance.
//
// In this pass we create additional cuts, at the expense of goal (1), with the
// aim of reducing overall register pressure to further goal (2) and improve
// performance overall.  Ideally, this pass would be integrated with the
// IR-level analysis but register pressure information is heavily
// target-dependent and liveness information is moreover not readily available
// at the IR level.  In short, we can't easily form the regions, utilizing
// accurate and effective aliasing information, and analyze liveness information
// both at the same points in LLVM's code generation flow.  Unfortunate, but
// fine for now, given my research goals.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "divide-machine-idempotent-regions"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/CalcSpillWeights.h"
#include "llvm/CodeGen/IdempotenceOptions.h"
#include "llvm/CodeGen/IdempotenceShadowIntervals.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineIdempotentRegions.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

#include "IdempotenceUtils.h"
#include "LiveDebugVariables.h"

using namespace llvm;

namespace {
  class DivideMachineIdempotentRegions : public MachineFunctionPass {
   public:
     static char ID;
     DivideMachineIdempotentRegions() : MachineFunctionPass(ID) {
       initializeDivideMachineIdempotentRegionsPass(
           *PassRegistry::getPassRegistry());
     }

     virtual void getAnalysisUsage(AnalysisUsage &AU) const;
     virtual void releaseMemory();
     virtual bool runOnMachineFunction(MachineFunction &MF);

   private:
     typedef std::vector<ShadowInterval *> ShadowsTy;
     ShadowsTy Shadows_;

     MachineFunction *MF_;
     IdempotenceShadowIntervals *ISI_;
     LiveIntervals *LIS_;
     LiveVariables *LVS_;
     MachineDominatorTree *MDT_;
     MachineIdempotentRegions *MIR_;
     MachineLoopInfo *MLI_;

     // Compute the shadow pressure at MI.
     unsigned computeShadowPressure(const MachineInstr &MI,
                                    ShadowsTy *Recompute) const;

     // Patch NewRegion so it doesn't loop back onto itself.
     void patchCycle(IdempotentRegion *NewRegion, ShadowsTy *Recompute);

     // Return an iterator to the point after all PHI destination copies and
     // region boundaries at the head of MBB.
     MachineInstr *skipBoundariesAndPHICopies(MachineBasicBlock *MBB) const {
       MachineBasicBlock::iterator I = MBB->begin();
       while (I != MBB->end() &&
              (MIR_->isRegionEntry(*I) || 
               (I->isCopy() && LVS_->isPHIJoin(I->getOperand(1).getReg()))))
         ++I;
       return I;
     }
  };
} // end anonymous namespace

static cl::opt<unsigned>
PressureThreshold("idempotence-pressure-threshold", cl::init(5), cl::Hidden,
  cl::desc("The maximum increase in liveness pressure due to idempotence"));

char DivideMachineIdempotentRegions::ID = 0;
INITIALIZE_PASS_BEGIN(DivideMachineIdempotentRegions,
                      "divide-machine-idempotence-regions",
                      "Divide Machine Idempotent Regions", false, false)
INITIALIZE_PASS_DEPENDENCY(CalculateSpillWeights)
INITIALIZE_PASS_DEPENDENCY(IdempotenceShadowIntervals)
INITIALIZE_PASS_DEPENDENCY(LiveVariables)
INITIALIZE_PASS_DEPENDENCY(LiveIntervals)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfo)
INITIALIZE_PASS_END(DivideMachineIdempotentRegions,
                    "divide-machine-idempotence-regions",
                    "Divide Machine Idempotent Regions", false, false)

char &llvm::DivideMachineIdempotentRegionsID =
    DivideMachineIdempotentRegions::ID;

void DivideMachineIdempotentRegions::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<CalculateSpillWeights>();
  AU.addRequired<IdempotenceShadowIntervals>();
  AU.addRequired<LiveIntervals>();
  AU.addRequired<LiveVariables>();
  AU.addRequired<MachineDominatorTree>();
  AU.addRequired<MachineIdempotentRegions>();
  AU.addRequired<MachineLoopInfo>();
  AU.addPreserved<IdempotenceShadowIntervals>();
  AU.addPreserved<LiveDebugVariables>();
  AU.addPreserved<LiveIntervals>();
  AU.addPreserved<LiveVariables>();
  AU.addPreserved<MachineDominatorTree>();
  AU.addPreserved<MachineIdempotentRegions>();
  AU.addPreserved<MachineLoopInfo>();
  AU.addPreserved<SlotIndexes>();
  AU.addPreservedID(MachineDominatorsID);
  AU.addPreservedID(PHIEliminationID);
  AU.addPreservedID(StrongPHIEliminationID);
  AU.addPreservedID(TwoAddressInstructionPassID);
  MachineFunctionPass::getAnalysisUsage(AU);
}

void DivideMachineIdempotentRegions::releaseMemory() {
  Shadows_.clear();
}

bool DivideMachineIdempotentRegions::runOnMachineFunction(MachineFunction &MF) {
  DEBUG(dbgs() << "********** DIVIDE MACHINE IDEMPOTENT REGIONS **********\n");
  assert(IdempotenceConstructionMode == IdempotenceOptions::OptimizeForSpeed &&
         "pass should not be run");

  MF_  = &MF;
  ISI_ = &getAnalysis<IdempotenceShadowIntervals>();
  LIS_ = &getAnalysis<LiveIntervals>();
  LVS_ = &getAnalysis<LiveVariables>();
  MDT_ = &getAnalysis<MachineDominatorTree>();
  MLI_ = &getAnalysis<MachineLoopInfo>();
  MIR_ = &getAnalysis<MachineIdempotentRegions>();

  // Pre-compute shadows and put into Shadows_.  This allows us direct access to
  // shadow information rather than having to go through LIS_ each time.
  for (LiveIntervals::iterator I = LIS_->begin(),
       IE = LIS_->end(); I != IE; ++I)
    Shadows_.push_back(&ISI_->getShadow(*I->second));

  // Walk the CFG in depth-first order with respect to the dominator tree so
  // that we are more likely to cut an MBB before we see any successor MBB
  // that may be affected and for which pressure may be reduced.
  bool Changed = false;
  ShadowsTy Recompute;
  SmallVector<MachineDomTreeNode*, 32> Worklist;
  Worklist.push_back(MDT_->getNode(MF_->begin()));
  do {
    MachineDomTreeNode *Node = Worklist.pop_back_val();
    assert(Node != NULL && "Null dominator tree node?");

    // Push DomTree children here in case of an early continue.
    const std::vector<MachineDomTreeNode *> &Children = Node->getChildren();
    for (unsigned I = 0; I < Children.size(); ++I)
      Worklist.push_back(Children[I]);

    // If this block is empty then skip it.  Otherwise, if we exceed the
    // pressure threshold, we will place a cut at the very beginning.
    MachineBasicBlock *MBB = Node->getBlock();
    MachineBasicBlock::iterator I = MBB->begin();
    if (I == MBB->end() || MIR_->isRegionEntry(*I))
      continue;

    // Compute the excess register pressure due to shadows.
    DEBUG(dbgs() << "Analyzing BB#" << MBB->getNumber() << "\n");
    Recompute.clear();
    unsigned ShadowPressure = computeShadowPressure(*I, &Recompute);

    // If not enough shadows contribute pressure then skip this block. 
    if (ShadowPressure < PressureThreshold)
      continue;

    // Otherwise insert a cut.
    DEBUG(dbgs() << "  Inserting boundary in BB#" << MBB->getNumber() << "\n");
    IdempotentRegion *NewRegion =
      &MIR_->createRegionBefore(MBB, I, LIS_->getSlotIndexes());
    Changed = true;

    // Check that the new region can't reach itself.  If it can, we need to
    // do some patching.  This patching is not as intelligent as in
    // PatchMachineIdempotentRegions since we don't have SSA (and hence
    // SSAUpdater) to help us out.  We just do pessimistic stuff.
    // FIXME:  Cut placement may be mostly redundant with neighboring cut.
    // Peephole optimizations possible.
    std::vector<IdempotentRegion *> Regions;
    MIR_->getRegionsContaining(NewRegion->getEntry(), &Regions);
    if (std::find(Regions.begin(), Regions.end(), NewRegion) != Regions.end())
      patchCycle(NewRegion, &Recompute);

    // Recompute affected shadows.
    DEBUG(dbgs() << "  Recomputing affected shadows...\n");
    for (ShadowsTy::iterator S = Recompute.begin(), SE = Recompute.end();
         S != SE; ++S)
      (*S)->recompute();

  } while (!Worklist.empty());

  // Reset spill weights so coalescing doesn't balk.
  for (LiveIntervals::iterator I = LIS_->begin(),
       IE = LIS_->end(); I != IE; ++I) {
    LiveInterval *LI = I->second;
    LI->weight = TargetRegisterInfo::isPhysicalRegister(LI->reg) ?
      HUGE_VALF : 0.0F;
  }

  return Changed;
}

static bool compareWeights(ShadowInterval *L, ShadowInterval *R) {
  return (L->getInterval().weight > R->getInterval().weight);
}

unsigned DivideMachineIdempotentRegions::computeShadowPressure(
   const MachineInstr &MI,
   ShadowsTy *Recompute) const {

  // Compute the shadows to potentially recompute.
  SlotIndex Slot = LIS_->getInstructionIndex(&MI).getBaseIndex();
  for (ShadowsTy::const_iterator S = Shadows_.begin(), SE = Shadows_.end();
       S != SE; ++S) {
    ShadowInterval *SI = *S;
    const LiveInterval *LI = &SI->getInterval();

    // If either the shadow will end at the boundary or the boundary may spawn
    // a shadow at the boundary then we must recompute the shadow.  
    if (SI->isShadowAt(Slot) || LI->liveAt(Slot))
      Recompute->push_back(SI);
  }

  // Among those shadows to recompute, count those among the 'heaviest' top 10
  // that are exerting shadow pressure.
  std::sort(Recompute->begin(), Recompute->end(), compareWeights);
  unsigned ShadowPressure = 0, I = 0;
  for (ShadowsTy::iterator S = Recompute->begin(), SE = Recompute->end();
       S != SE && I < 10; ++S, ++I)
    if ((*S)->isShadowAt(Slot))
      ShadowPressure++;

  return ShadowPressure;
}

void DivideMachineIdempotentRegions::patchCycle(IdempotentRegion *NewRegion,
                                                ShadowsTy *Recompute) {
  // There is a cycle and we must break it.  If MBB is the loop header then just
  // insert a boundary after the first-non-PHI-copy.  Otherwise also insert one
  // at the very front of the header block to avoid loop back-edge clobbers.
  MachineBasicBlock *MBB = &NewRegion->getEntryMBB();
  MachineLoop *Loop = MLI_->getLoopFor(MBB);
  assert(Loop != NULL && "Cycle in a non-natural loop not already handled");

  SmallVector<MachineBasicBlock::iterator, 2> CutPoints;
  MachineBasicBlock *Header = Loop->getHeader();
  CutPoints.push_back(skipBoundariesAndPHICopies(Header));
  if (MBB != Header)
    CutPoints.push_back(Header->begin());

  // Insert the one or two cuts.
  for (unsigned Idx = 0; Idx < CutPoints.size(); ++Idx) {
    MachineBasicBlock::iterator I = CutPoints[Idx];
    MIR_->createRegionBefore(Header, I, LIS_->getSlotIndexes());

    // Compute affected shadows.
    SlotIndex Slot = LIS_->getInstructionIndex(I).getBaseIndex();
    for (ShadowsTy::const_iterator S = Shadows_.begin(), SE = Shadows_.end();
         S != SE; ++S) {
      ShadowInterval *SI = *S;
      const LiveInterval *LI = &SI->getInterval();
      if (SI->isShadowAt(Slot) || LI->liveAt(Slot))
        Recompute->push_back(SI);
    }
  }
}

