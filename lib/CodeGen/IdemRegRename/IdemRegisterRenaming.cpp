//===----- IdemRegisterRenaming.cpp - Register regnaming after RA ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "reg-renaming"

#include <llvm/PassSupport.h>
#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "LiveIntervalAnalysisIdem.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/Target/TargetData.h"
#include "IdemUtil.h"

#include <algorithm>
#include <iterator>
#include <deque>
#include <utility>
#include <algorithm>

using namespace llvm;

/// @author Jianping Zeng.
namespace {

struct AntiDepPair {
  MachineOperand *use;
  MachineOperand *def;

  // like R0 = R0 + R0
  // the first pair is <def R0, first use R0>
  // second pair(only second R0) will stores in useInSameMI.
  std::vector<MachineOperand *> usesInSameMI;

  AntiDepPair(MachineOperand *_use, MachineOperand *_def) : use(_use), def(_def) {}

  bool operator==(AntiDepPair rhs) {
    return use == rhs.use && def == rhs.def;
  }

  bool operator!=(AntiDepPair rhs) {
    return !(*this == rhs);
  }

  bool operator<(const AntiDepPair &rhs) {
    if ((void *) use < (void *) rhs.use)
      return true;
    else
      return (void *) def < (void *) rhs.def;
  }

  bool operator<(const AntiDepPair &rhs) const {
    return *const_cast<AntiDepPair *>(this) < rhs;
  }
};

class RegisterRenaming : public MachineFunctionPass {
public:
  static char ID;
  RegisterRenaming() : MachineFunctionPass(ID) {
    initializeRegisterRenamingPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnMachineFunction(MachineFunction &MF) override;
  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<LiveIntervalAnalysisIdem>();
    AU.addRequired<MachineIdempotentRegions>();
    AU.addRequired<MachineDominatorTree>();
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
  const char *getPassName() const {
    return "Register Renaming for Idempotence pass";
  }
  virtual void releaseMemory() {
    prevDefRegs.clear();
    prevUseRegs.clear();
    antiDeps.clear();
    regions->releaseMemory();
    sequence.clear();
    //delete scavenger;
  }
private:
  /**
   * This method should be called to respond to re-construct idempotence and
   * live interval information.
   */
  void reconstructIdemAndLiveInterval() {
    regions->releaseMemory();
    regions->runOnMachineFunction(*const_cast<MachineFunction *>(mf));
    li->releaseMemory();
    li->runOnMachineFunction(*const_cast<MachineFunction *>(mf));
  }

  bool regionContains(IdempotentRegion *region, MachineInstr *mi);

  bool regionContains(SmallVectorImpl<IdempotentRegion *> *Regions, MachineInstr *mi);

  template<bool IgnoreIdem = false>
  void computeDefUseDataflow(MachineInstr *mi,
                             std::set<MachineOperand *> &uses,
                             SmallVectorImpl<IdempotentRegion *> *Regions,
                             std::map<MachineInstr *, std::set<MachineOperand *>> &prevDefs,
                             std::map<MachineInstr *, std::set<MachineOperand *>> &prevUses);
  inline void addAntiDeps(MachineOperand *useMO, MachineOperand *defMO);
  void collectRefDefUseInfo(MachineInstr *mi, SmallVectorImpl<IdempotentRegion *> *Regions);
  /**
   * Checks if the MachineInstr is two address instruction or not. Return true if it is.
   * @param useMI
   * @return
   */
  bool isTwoAddressInstr(MachineInstr *useMI);
  bool shouldRename(AntiDepPair pair);
  void filterUnavailableRegs(MachineOperand *use, BitVector &allocSet,
                             bool allowsAntiDep = false);
  unsigned choosePhysRegForRenaming(MachineOperand *use,
                                    LiveIntervalIdem *interval);
  unsigned tryChooseFreeRegister(LiveIntervalIdem &interval,
                                 int useReg,
                                 BitVector &allocSet);
  /**
   * Checks if it is legal to replace reg with physReg.
   * @param physReg
   * @param reg
   * @return
   */
  inline bool legalToReplace(unsigned physReg, int reg);

  unsigned tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                    int useReg,
                                    BitVector &allocSet);

  void spillOutInterval(LiveIntervalIdem *interval);

  void simplifyAntiDeps();

//  void clearAntiDeps(MachineOperand *useMO);

  void updatePrevDefUses();

  void insertMoveAndBoundary(AntiDepPair &pair, std::vector<MachineOperand*> *uses = 0);

  void getCandidateInsertionPositionsDFS(MachineInstr *startPos,
                                         std::set<MachineInstr *> &candidates,
                                         std::set<MachineBasicBlock *> &visited);

  void getCandidatePosForBoundaryInsert(MachineInstr *startPos,
                                        std::set<MachineInstr *> &candidates,
                                        std::set<MachineBasicBlock *> &visited);

  typedef std::pair<MachineBasicBlock::reverse_iterator, MachineBasicBlock::reverse_iterator> ItrRange;

  void computeDistance(ItrRange range,
                       std::set<MachineInstr *> &candidates,
                       std::map<MachineInstr *, unsigned> &dists,
                       unsigned distance,
                       std::set<MachineBasicBlock *> &visited);

  void computeOptimizedInsertion(MachineInstr *startPos,
                                 std::set<MachineInstr *> &candidates,
                                 std::vector<MachineInstr *> &InsertedPos);

  bool handleMultiDepsWithinSameMI(AntiDepPair &pair);

  bool scavengerIdem();

  bool idemCanBeRemoved(MachineInstr *mi);

  void dump() {
    for (auto pair : antiDeps) {
      auto def = pair.def;
      auto use = pair.use;
      llvm::errs() << "[" << li->mi2Idx[def->getParent()] << ", " <<
                   li->mi2Idx[use->getParent()] <<
                   ", " << tri->getName(def->getReg()) << "]\n";
    }

    li->dump(sequence);
  }

  /**
   * This method acts as a special method used to deal with the single
   * def and multiple uses on same register, depicted as follows.
   * @code
   * ... = R1[first use] + 11
   * ...
   * R1[def] = ADDri R1[second use] + 10
   * @endcode
   *
   * Prior code should be morphyed by inserting only one boundary without
   * one redundant boundary instr, so resulted generated will be.
   * @code
   * R2 = R1
   *    ...
   * ... = R2 + 11
   *    ...
   * R1[def] = ADDri R2 + 10
   * @endcode
   * @return Return true if we do successful transformation.
   */
  bool handleSingleDefMultiUses();

  // records all def registers by instr before current mi.
  std::map<MachineInstr *, std::set<MachineOperand *>> prevDefRegs;
  // records all use registers of current mi and previous mi.
  std::map<MachineInstr *, std::set<MachineOperand *>> prevUseRegs;
  std::deque<AntiDepPair> antiDeps;
  const TargetInstrInfo *tii;
  const TargetRegisterInfo *tri;
  const MachineFunction *mf;
  LiveIntervalAnalysisIdem *li;
  MachineFrameInfo *mfi;
  MachineRegisterInfo *mri;
  const TargetData *td;
  MachineIdempotentRegions *regions;
  BitVector allocaSet;
  /**
   * Used for cleaning some redundant idem call instruction after register renaming.
   */
  IdemInstrScavenger *scavenger;
  MachineDominatorTree *dt;
  std::vector<MachineBasicBlock *> sequence;
};
}

char RegisterRenaming::ID = 0;

INITIALIZE_PASS_BEGIN(RegisterRenaming, "reg-renaming",
                      "Register Renaming for Idempotence", false, false)
  INITIALIZE_PASS_DEPENDENCY(LiveIntervalAnalysisIdem)
  INITIALIZE_PASS_DEPENDENCY(MachineIdempotentRegions)
  INITIALIZE_PASS_DEPENDENCY(MachineDominatorTree)
INITIALIZE_PASS_END(RegisterRenaming, "reg-renaming",
                    "Register Renaming for Idempotence", false, false)

FunctionPass *llvm::createRegisterRenamingPass() {
  return new RegisterRenaming();
}

//=== Implementation for class RegisterRenaming.  ====//

/**
 * Checks if the given element is contained in the range from first to end.
 * @tparam _ForwardIterator
 * @tparam _BinaryPredicate
 * @param first
 * @param end
 * @param p
 * @return
 */
template<class T, typename _BinaryPredicate>
bool contain(std::set<T> &set, T mo, _BinaryPredicate p) {
  auto itr = set.begin();
  auto end = set.end();
  for (; itr != end; ++itr)
    if (p(*itr, mo))
      return true;

  return false;
}

template<class T, class _Binary_Predicate>
static std::set<T> Union(std::set<T> &lhs, std::set<T> &rhs, _Binary_Predicate pred) {
  std::set<T> res;
  for (T t : lhs) {
    if (!contain(res, t, pred))
      res.insert(t);
  }
  for (T t : rhs) {
    if (!contain(res, t, pred))
      res.insert(t);
  }
  return res;
}

template<class T>
static std::set<T> Union(std::set<T> &lhs, std::set<T> &rhs) {
  std::set<T> res;
  for (auto e : lhs)
    res.insert(e);
  for (auto e : rhs)
    res.insert(e);
  //res.insert(lhs.begin(), lhs.end());
  //res.insert(rhs.begin(), rhs.begin());
  return res;
}

template<class T>
static void intersect(std::set<T> &res, std::set<T> lhs, std::set<T> rhs) {
  for (T elt : lhs) {
    if (!rhs.count(elt))
      res.insert(elt);
  }
}

static void getDefUses(MachineInstr *mi,
                       std::set<MachineOperand *> *defs,
                       std::set<MachineOperand *> *uses,
                       const BitVector &allocaSets) {
  for (unsigned i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand *mo = &mi->getOperand(i);
    if (!mo || !mo->isReg() ||
        !mo->getReg() || mo->isImplicit() ||
        !allocaSets.test(mo->getReg()))
      continue;

    unsigned &&reg = mo->getReg();
    assert(TargetRegisterInfo::isPhysicalRegister(reg));

    if (mo->isDef() && defs) {
      defs->insert(mo);
    } else if (mo->isUse() && uses)
      uses->insert(mo);
  }
}

bool contains(IdempotentRegion::inst_iterator begin,
              const IdempotentRegion::inst_iterator &end,
              MachineInstr *elt) {
  auto itr = std::move(begin);
  while (itr != end) {
    if (*itr == elt)
      return true;
    ++itr;
  }
  return false;
}

MachineInstr *getPrevMI(MachineInstr *mi) {
  if (!mi || !mi->getParent())
    return nullptr;
  return ilist_traits<MachineInstr>::getPrev(mi);
}

MachineInstr *getNextMI(MachineInstr *mi) {
  if (!mi || !mi->getParent())
    return nullptr;
  return ilist_traits<MachineInstr>::getNext(mi);
}

inline static bool predEq(const MachineOperand *o1, const MachineOperand *o2) { return o1->getReg() == o2->getReg(); }

template<typename T, typename _InputIterator>
std::pair<_InputIterator, bool> contains(_InputIterator first, _InputIterator last, T t) {

  for (; first != last; ++first) {
    if (*first == t)
      return std::pair<_InputIterator, bool>(first, true);
  }
  return std::pair<_InputIterator, bool>(_InputIterator(), false);
}
bool RegisterRenaming::regionContains(IdempotentRegion *region,
                                      MachineInstr *mi) {
  if (!region || !mi)
    return false;

  auto mbbItr = region->mbb_begin();
  auto mbbEnd = region->mbb_end();
  while (mbbItr != mbbEnd) {
    auto miItr = (*mbbItr).first;
    auto miEnd = (*mbbItr).second;
    for (; miItr != miEnd; ++miItr) {
      if (&*miItr == mi)
        return true;
    }
    ++mbbItr;
  }
  return false;
}

bool RegisterRenaming::regionContains(SmallVectorImpl<IdempotentRegion *> *Regions,
                                      MachineInstr *mi) {
  for (IdempotentRegion *r : *Regions) {
    if (regionContains(r, mi))
      return true;
  }
  return false;
}
/**
 * This function will be used for computing the previous defs and uses for each MI.
 * In the same time, template argument {@code IgnoreIdem} will determines if
 * should we set the defs and uses set of idem as empty or as normal mi.
 * @tparam IgnoreIdem
 * @param mi
 * @param prevDefs
 * @param prevUses
 */
template<bool IgnoreIdem>
void RegisterRenaming::computeDefUseDataflow(MachineInstr *mi,
                                             std::set<MachineOperand *> &uses,
                                             SmallVectorImpl<IdempotentRegion *> *Regions,
                                             std::map<MachineInstr *, std::set<MachineOperand *>> &prevDefs,
                                             std::map<MachineInstr *, std::set<MachineOperand *>> &prevUses) {
  if (regions->isRegionEntry(*mi) && !IgnoreIdem) {
    prevDefs[mi] = std::set<MachineOperand *>();
    prevUses[mi] = uses;
  } else {
    // if the mi is the first mi of basic block with preds.
    if (mi == &mi->getParent()->front()) {
      MachineBasicBlock *mbb = mi->getParent();
      if (mbb->pred_empty()) {
        prevDefs[mi] = std::set<MachineOperand *>();
        prevUses[mi] = uses;
      } else {
        std::set<MachineOperand *> &predDefs = prevDefs[mi];
        std::set<MachineOperand *> &predUses = prevUses[mi];

        auto itr = mbb->pred_begin();
        auto end = mbb->pred_end();
        for (; itr != end; ++itr) {
          MachineBasicBlock *pred = *itr;
          if (pred->empty())
            continue;

          MachineInstr *predMI = &pred->back();
          if (!regionContains(Regions, predMI))
            continue;

          std::set<MachineOperand *> localUses;
          std::set<MachineOperand *> localDefs;

          getDefUses(predMI, &localDefs, &localUses, allocaSet);

          predDefs = Union(localDefs, prevDefs[predMI], predEq);
          predUses = Union(localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    } else {
      // otherwise
      std::set<MachineOperand *> localPrevDefs;
      MachineInstr *prevMI = getPrevMI(mi);
      //IDEM_DEBUG(prevMI->dump());

      assert(prevMI && "previous machine instr can't be null!");
      getDefUses(prevMI, &localPrevDefs, 0, allocaSet);

      prevDefs[mi] = Union(prevDefs[prevMI], localPrevDefs, predEq);
      prevUses[mi] = Union(prevUses[prevMI], uses);

      /*
      IDEM_DEBUG(for (auto def : prevDefs[mi])
        llvm::errs()<<tri->getName(def->getReg())<<" ";
      llvm::errs()<<"\n";
      for (auto use : prevUses[mi])
        llvm::errs()<<tri->getName(use->getReg())<<" ";
      llvm::errs()<<"\n";);*/
    }
  }
}

inline void RegisterRenaming::addAntiDeps(MachineOperand *useMO, MachineOperand *defMO) {
  AntiDepPair res = {useMO, defMO};
  if (!contains(antiDeps.begin(), antiDeps.end(), res).second)
    antiDeps.push_back(res);
}

void RegisterRenaming::collectRefDefUseInfo(MachineInstr *mi,
                                            SmallVectorImpl<IdempotentRegion *> *Regions) {
  if (!mi)
    return;
  std::set<MachineOperand *> uses;
  std::set<MachineOperand *> defs;
  getDefUses(mi, &defs, &uses, allocaSet);

  computeDefUseDataflow(mi, uses, Regions, prevDefRegs, prevUseRegs);

  //
  // unique the defs register operand by checking if it have same register.
  // (void)std::unique(defs->begin(), defs->end(),
  //                  [](MachineOperand* def1, MachineOperand* def2)
  //                  { return def1->getReg() == def2->getReg(); });

  if (defs.empty())
    return;
  std::set<MachineOperand *> &prevUses = prevUseRegs[mi];

  for (auto defMO : defs) {
    for (MachineOperand *mo : prevUses) {

      // We should not collect pair about two address instruction.
      if (isTwoAddressInstr(mo->getParent()))
        continue;

      // we don't care those anti-dependence whose def and use are not  belong to
      // the same idempotence region.
      if (!regionContains(Regions, mo->getParent()))
        continue;
      if (mo->isReg() && mo->getReg() == defMO->getReg() &&
          !contain(prevDefRegs[mo->getParent()], mo, predEq)) {
        addAntiDeps(mo, defMO);
      }
    }
  }
}

bool RegisterRenaming::isTwoAddressInstr(MachineInstr *useMI) {
  // We should not rename the two-address instruction.
  auto MCID = useMI->getDesc();
  int numOps = useMI->isInlineAsm() ? useMI->getNumOperands() : MCID.getNumOperands();
  for (int i = 0; i < numOps; i++) {
    unsigned destIdx;
    if (!useMI->isRegTiedToDefOperand(i, &destIdx))
      continue;

    return true;
  }
  return false;
}

bool RegisterRenaming::shouldRename(AntiDepPair pair) {
  auto use = pair.use;
  MachineInstr *useMI = use->getParent();
  std::set<MachineOperand *> &defs = prevDefRegs[useMI];

  // We should not rename the two-address instruction.
  if (isTwoAddressInstr(useMI))
    return false;

  return !contain(defs, use, predEq);
}

void RegisterRenaming::spillOutInterval(LiveIntervalIdem *interval) {
  int frameIndex = INT_MIN;
  for (auto itr = interval->usepoint_begin(), end = interval->usepoint_end();
       itr != end; ++itr) {
    MachineOperand *mo = itr->mo;
    MachineInstr *mi = mo->getParent();
    assert(mo->isReg());
    const TargetRegisterClass *rc = tri->getMinimalPhysRegClass(mo->getReg());
    if (mo->isDef()) {
      frameIndex = mfi->CreateSpillStackObject(rc->getSize(), rc->getAlignment());
      auto st = getNextMI(mi);
      tii->storeRegToStackSlot(*mi->getParent(), st,
                               mo->getReg(), false, frameIndex, rc, tri);

      auto copyMI = getNextMI(mi);
      for (int i = copyMI->getNumOperands()-1; i >= 0; --i)
        if (copyMI->getOperand(i).isReg() && copyMI->getOperand(i).getReg() == mo->getReg()) {
          copyMI->getOperand(i).setIsUndef(true);
          break;
        }

    } else if (mo->isUse()) {
      assert(frameIndex != INT_MIN);
      tii->loadRegFromStackSlot(*mi->getParent(), mi, mo->getReg(), frameIndex, rc, tri);
      // Inserts a boundary instruction immediately before the load to partition the
      // region into two different parts for avoiding violating idempotence.
      auto ld = getPrevMI(mi);
      tii->emitIdemBoundary(*mi->getParent(), ld);
    }
  }
}

bool intersects(BitVector lhs, BitVector rhs) {
  for (int idx = lhs.find_first(); idx != -1; idx = lhs.find_next(idx))
    if (!rhs[idx])
      return false;

  return true;
}

bool RegisterRenaming::legalToReplace(unsigned physReg, int reg) {
  for (unsigned i = 0, e = tri->getNumRegClasses(); i < e; i++) {
    auto rc = tri->getRegClass(i);
    if (rc->contains(physReg) && rc->contains(reg))
      return true;
  }
  return false;

  /*llvm::errs()<<tri->getMinimalPhysRegClass(physReg)->getName()<<", "<< rc.getName()<<"\n";
  return tri->getMinimalPhysRegClass(physReg)->hasSubClassEq(&rc) *//*&& !mri->isLiveIn(physReg)*//*;*/
}

unsigned RegisterRenaming::tryChooseFreeRegister(LiveIntervalIdem &interval,
                                                 int useReg,
                                                 BitVector &allocSet) {
  IDEM_DEBUG(llvm::errs() << "Interval for move instr: ";
                 interval.dump(*const_cast<TargetRegisterInfo *>(tri));
                 llvm::errs() << "\n";);

  for (int physReg = allocSet.find_first(); physReg > 0; physReg = allocSet.find_next(physReg)) {
    if (li->intervals.count(physReg)) {
      LiveIntervalIdem *itrv = li->intervals[physReg];

      IDEM_DEBUG(llvm::errs() << "Candidate interval: ";
                     itrv->dump(*const_cast<TargetRegisterInfo *>(tri));
                     llvm::errs() << "\n";);

      if (!itrv->intersects(&interval)) {
        // we only consider those live interval which doesn't interfere with current
        // interval.
        // No matching in register class should be ignored.
        // Avoiding LiveIn register(such as argument register).
        if (!legalToReplace(physReg, useReg))
          continue;

        return physReg;
      }
    } else if (legalToReplace(physReg, useReg)) {
      // current physReg is free, so return it.
      return static_cast<unsigned int>(physReg);
    }
  }
  return 0;
}

unsigned RegisterRenaming::tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                                    int useReg,
                                                    BitVector &allocSet) {
  // choose an interval to be evicted into memory, and insert spilling code as
  // appropriate.
  unsigned costMax = INT_MAX;
  LiveIntervalIdem *targetInter = nullptr;
  for (auto physReg = allocSet.find_first(); physReg > 0;
       physReg = allocSet.find_next(physReg)) {
    if (!legalToReplace(physReg, useReg))
      continue;
    assert(li->intervals.count(physReg) && "Why tryChooseFreeRegister does't return it?");
    auto phyItv = li->intervals[physReg];

    IDEM_DEBUG(llvm::errs()<<"Found: "<< tri->getMinimalPhysRegClass(physReg)<<"\n";);
    // TargetRegisterClass isn't compatible with each other.
    if (!legalToReplace(phyItv->reg, useReg))
      continue;

    if (mri->isLiveIn(physReg))
      continue;

    assert(interval.intersects(phyItv) &&
        "should not have interval doesn't interfere with current interval");

    if (phyItv->costToSpill < costMax) {
      costMax = phyItv->costToSpill;
      targetInter = phyItv;
    }
  }

  // no proper interval found to be spilled out.
  if (!targetInter)
    return 0;

  IDEM_DEBUG(llvm::errs() << "Selected evicted physical register is: "
                          << tri->getName(targetInter->reg) << "\n";
                 llvm::errs() << "\nSelected evicted interval is: ";
                 targetInter->dump(*const_cast<TargetRegisterInfo *>(tri)););

  spillOutInterval(targetInter);
  return targetInter->reg;
}

void RegisterRenaming::filterUnavailableRegs(MachineOperand *use,
                                            BitVector &allocSet,
                                            bool allowsAntiDep) {

  /*IDEM_DEBUG(for (int i = allocSet.find_first(); i > 0; i = allocSet.find_next(i)) {
    llvm::errs()<<tri->getName(i)<<" ";
  }
  llvm::errs()<<"\n";);
  */
  // remove the defined register by use mi from allocable set.
  std::set<MachineOperand *> defs;
  getDefUses(use->getParent(), &defs, 0, tri->getAllocatableSet(*mf));
  for (MachineOperand *phy : defs)
    allocSet[phy->getReg()] = false;

  // also, we must make sure no the physical register same as
  // use will be assigned.
  allocSet[use->getReg()] = false;

  // Remove some physical register whose register class is not compatible with rc.
  // const TargetRegisterClass *rc = tri->getMinimalPhysRegClass(use->getReg());
  for (int physReg = allocSet.find_first(); physReg != -1; physReg = allocSet.find_next(physReg))
    if (!legalToReplace(physReg, use->getReg()))
      allocSet[physReg] = false;

  // When we use another new register to replace the used register,
  // we may introduce new anti-dependence between new reg and defined register
  // by successive instr.
  //
  // So we need a flag tell us whether anti-dependences occur is allowed or not.
  // if it is allowed, so we should re-handle it after.
  //
  // Before:
  // R0 = R0 + R1
  //    ...
  // R2 = ...
  //
  // After:
  // R2 = R0
  //---------
  // R0 = R2 + R1
  //    ...
  // R2 = ...
  // So anti-dependence occurs again !!!
  if (!allowsAntiDep) {
    std::vector<MachineInstr *> worklist;
    worklist.push_back(use->getParent());
    std::set<MachineBasicBlock *> visited;

    while (!worklist.empty()) {
      MachineInstr *startPos = worklist.back();
      worklist.pop_back();

      // Also the assigned register can not is same as the defined reg by successive instr.
      MachineBasicBlock::iterator itr(startPos);
      auto mbb = itr->getParent();
      if (!visited.insert(mbb).second)
        continue;

      for (++itr; itr != mbb->end() && !tii->isIdemBoundary(itr); ++itr) {
        std::set<MachineOperand *> defs;
        getDefUses(itr, &defs, 0, allocSet);
        for (auto defMO : defs)
          allocSet[defMO->getReg()] = false;
      }

      if (itr != mbb->end())
        return;

      std::for_each(mbb->succ_begin(), mbb->succ_end(), [&](MachineBasicBlock *succ) {
        worklist.push_back(&succ->front());
      });
    }
  }
}

unsigned RegisterRenaming::choosePhysRegForRenaming(MachineOperand *use,
                                                    LiveIntervalIdem *interval) {
  auto rc = tri->getMinimalPhysRegClass(use->getReg());
  auto allocSet = tri->getAllocatableSet(*mf);
  IDEM_DEBUG(llvm::errs()<<"Required: "<<rc->getName()<<"\n";);

  // Remove some registers are not available when making decision of choosing.
  filterUnavailableRegs(use, allocSet);

  // obtains a free register used for move instr.
  unsigned useReg = use->getReg();
  unsigned freeReg = tryChooseFreeRegister(*interval, useReg, allocSet);
  if (!freeReg) {
    freeReg = tryChooseBlockedRegister(*interval, useReg, allocSet);
  }

  // If until now, we found no free register, so try to enable flag 'allowsAntiDep'
  // and gives a chance re-handle it.
  if (!freeReg) {
    allocSet = tri->getAllocatableSet(*mf);

    // Remove some registers are not available when making decision of choosing.
    filterUnavailableRegs(use, allocSet, true);

    IDEM_DEBUG(llvm::errs() << "Required: " << tri->getMinimalPhysRegClass(use->getReg())->getName() << "\n";);
    // obtains a free register used for move instr.
    freeReg = tryChooseFreeRegister(*interval, useReg, allocSet);
    if (!freeReg) {
      freeReg = tryChooseBlockedRegister(*interval, useReg, allocSet);
    }
  }

  assert(freeReg && "can not to rename the specified register!");
  interval->reg = freeReg;
  li->insertOrCreateInterval(freeReg, interval);
  return freeReg;
}

void RegisterRenaming::simplifyAntiDeps() {
  // Step#5: checks if we should rename the defined register according to usd-def,
  //         If it is, construct a pair of use-def reg pair.

  // remove some equivalent anti-dependence, like
  //    |----|
  //    v    v
  //   r0 = r0 + r1
  //       /
  //     /
  //   r0 = r0 + 1
  for (size_t i = 0, e = antiDeps.size(); i < e; i++) {
    auto target = antiDeps[i];
    for (size_t j = i + 1; j < e; ++j) {
      auto pair = antiDeps[j];
      assert(pair.use->getParent() && pair.use->getParent()->getParent() &&
          pair.def->getParent() && pair.def->getParent()->getParent());

      assert(pair.use->isReg() && pair.def->isReg());
      if (pair.use == target.use && pair.def != target.def &&
          pair.def->getReg() == target.def->getReg()) {
        // we can remove pair from antiDep list.
        antiDeps[j] = antiDeps[e - 1];
        antiDeps.pop_back();
        --j;
        --e;
      }
    }
  }

  for (size_t i = 0, e = antiDeps.size(); i < e; i++) {
    auto target = antiDeps[i];
    for (size_t j = i + 1; j < e; ++j) {
      auto pair = antiDeps[j];
      assert(pair.use->getParent() && pair.use->getParent()->getParent() &&
          pair.def->getParent() && pair.def->getParent()->getParent());

      assert(pair.use->isReg() && pair.def->isReg());

      if (antiDeps[j].def == antiDeps[i].def &&
          antiDeps[j].use->getReg() == antiDeps[i].use->getReg() &&
          antiDeps[j].use->getParent() == antiDeps[i].use->getParent()) {
        // R0 = R0 + R0  (two same anti-dependences)
        antiDeps[i].usesInSameMI.push_back(antiDeps[j].use);

        // delete element at index j.
        antiDeps[j] = antiDeps[e - 1];
        antiDeps.pop_back();
        --j;
        --e;
      }
    }
  }
}

/*
#if 0
void RegisterRenaming::insertBoundaryAsNeed(MachineInstr *&useMI,
                                            unsigned defReg) {
  MachineBasicBlock *mbb = useMI->getParent();
  assert(mbb);

  bool boundaryExists = false;
  auto begin = mbb->rbegin();
  auto end = mbb->rend();
  MachineBasicBlock::reverse_instr_iterator itr;
  for (itr = begin; itr != end && &*itr != useMI; ++itr) {}
  ++itr;

  while (itr != end) {
    if (tii->isIdemBoundary(&*itr)) {
      boundaryExists = true;
      break;
    }
    auto defMO = itr->getOperand(0);
    if (defMO.isReg() && defMO.getReg() && defMO.getReg() == defReg) {
      // set is not needed.
      // boundaryExists = false;
      break;
    }
    ++itr;
  }
  // Insert a boundary as needed.
  if (!boundaryExists) {
    tii->emitIdemBoundary(*mbb, useMI);
    useMI = --MachineBasicBlock::instr_iterator(useMI);
  }
  else
    useMI = &*itr;
}

#endif

void RegisterRenaming::clearAntiDeps(MachineOperand *useMO) {
  auto itr = antiDeps.begin();
  auto end = antiDeps.end();
  while (itr != end) {
    auto pair = *itr;
    if (pair.use == useMO) {
      // we find a anti-dep whose use is same as useMO.
      antiDeps.erase(itr);
    } else
      ++itr;
  }
}*/

void RegisterRenaming::updatePrevDefUses() {
  // FIXME, We need to update LiveIntervalAnalysis result caused by inserting move and spill code
  reconstructIdemAndLiveInterval();

  // FIXME, 9/17/2018, we need update prevDef, prevUses reg set, and idempotence regions.
  for (auto &mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto mie = mbb->instr_end();
    for (; mi != mie; ++mi) {
      // Step#3: collects reg definition information.
      // Step#4: collects reg uses information.
      SmallVectorImpl<IdempotentRegion *> Regions(10);
      regions->getRegionsContaining(*mi, &Regions);

      std::set<MachineOperand *> uses;
      std::set<MachineOperand *> defs;
      getDefUses(mi, &defs, &uses, allocaSet);

      computeDefUseDataflow(&*mi, uses, &Regions, prevDefRegs, prevUseRegs);
    }
  }
}

void RegisterRenaming::insertMoveAndBoundary(AntiDepPair &pair,
                                            std::vector<MachineOperand*> *uses) {
  auto useMI = pair.use->getParent();

  // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
  //insertBoundaryAsNeed(useMI, phyReg);
  MachineBasicBlock *mbb = 0;

  // Find the candiate insertion positions.
  std::set<MachineInstr *> candidateInsertPos;
  std::set<MachineBasicBlock *> visited;
  getCandidatePosForBoundaryInsert(useMI, candidateInsertPos, visited);

  IDEM_DEBUG(llvm::errs() << "\n    ************** Handles Anti-deps [" <<
               li->mi2Idx[pair.def->getParent()] << ", " <<
               li->mi2Idx[pair.use->getParent()] << ", " <<
               tri->getName(pair.use->getReg()) << "] **************\n";

  llvm::errs() << "candidate insertion positions:\n";
  for (auto mi : candidateInsertPos) {
    llvm::errs() << li->mi2Idx[mi] << ": ";
    mi->dump();
  });

  // Computes the optimizing positions.
  std::vector<MachineInstr *> optInsertedPos;
  computeOptimizedInsertion(useMI, candidateInsertPos, optInsertedPos);
  assert(optInsertedPos.size() == 1 && "should have only one optimal inserted position");

  // insert a boundary right after the use MI for avoiding validating the idem region
  // after use MI.

  // Inserts a poir of boundary and move instrs at each insertion point.
  auto pos = optInsertedPos[0];
  IDEM_DEBUG(llvm::errs() << "Inserted position:\n";
                 llvm::errs() << li->mi2Idx[pos] << ": ";
                 pos->dump(););

  assert(pos && pos->getParent() &&
      pos->getParent()->getParent() == mf);

  mbb = pos->getParent();

  // FIXME, 9/17/2018. Now, live range computes correctly .
  LiveIntervalIdem *interval = new LiveIntervalIdem;
  auto from = li->getIndex(pos) - 2;
  auto to = li->getIndex(useMI);
  unsigned int maxTo = to;
  if (uses) {
    for (MachineOperand *mo : *uses) {
      auto id = li->mi2Idx[mo->getParent()];
      if (id > maxTo)
        maxTo = id;
      interval->addUsePoint(id, mo);
    }
  }
  to = maxTo;
  interval->addRange(from, to);    // add an interval for a temporal move instr.
  unsigned phyReg = choosePhysRegForRenaming(pair.use, interval);

  assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
  assert(phyReg != pair.use->getReg());

  // Step#8: substitute the old reg with phyReg,
  // and remove other anti-dep on this use.
  unsigned oldReg = pair.use->getReg();
  //clearAntiDeps(pair.use);

  pair.use->setReg(phyReg);

  if (!pair.usesInSameMI.empty())
    std::for_each(pair.usesInSameMI.begin(), pair.usesInSameMI.end(),
                  [=](MachineOperand *p) { p->setReg(phyReg); });

  // Inserts two boundary instruction to surround the move instr.
  tii->emitIdemBoundary(*mbb, pos);
  tii->emitIdemBoundary(*mbb, pos);
  pos = getPrevMI(pos);

  assert(tii->isIdemBoundary(pos) && "the mi at inserted position must be a splitting boundary!");

  // Step#10: insert a move instruction before splitting boundary instr.
  // This instruction would be the last killer of src reg in this copy instr.
  tii->copyPhysReg(*mbb, pos, DebugLoc(), phyReg, oldReg, false);

  // annotate the undef flag to the src reg if src reg is liveIn.
  auto copyMI = getPrevMI(pos);

  // attach the undef flag to all src regs.
  for (unsigned i = 1, e = copyMI->getNumOperands(); i < e; ++i) {
    auto &mo = copyMI->getOperand(i);
    if (mo.isReg() && mo.getReg())
      mo.setIsUndef(true);
  }

  // FIXME, 9/17/2018, we need update prevDef, prevUses reg set, and idempotence regions.
  updatePrevDefUses();
/*  IDEM_DEBUG(llvm::errs() << "After inserted move instruction:\n";
                 mf->dump(););*/
}

void RegisterRenaming::getCandidateInsertionPositionsDFS(MachineInstr *startPos,
                                                         std::set<MachineInstr *> &candidates,
                                                         std::set<MachineBasicBlock *> &visited) {
  assert(startPos && startPos->getParent());

  MachineBasicBlock::iterator itr = startPos;
  auto mbb = startPos->getParent();

  if (!visited.insert(mbb).second)
    return;

  auto end = startPos->getParent()->end();
  for (; itr != end && !tii->isIdemBoundary(itr); ++itr) {
    std::set<MachineOperand *> defs;
    SmallVectorImpl<IdempotentRegion *> Regions(20);
    regions->getRegionsContaining(*itr, &Regions);

    getDefUses(itr, &defs, 0, allocaSet);
    auto prevUses = prevUseRegs[itr];
    for (auto defMO : defs) {
      for (MachineOperand *useMO : prevUses) {
        if (!predEq(defMO, useMO))
          continue;

        if (!regionContains(&Regions, useMO->getParent()))
          continue;
        for (auto redefReg : prevDefRegs[useMO->getParent()]) {
          if (regionContains(&Regions, useMO->getParent()) &&
              predEq(redefReg, useMO))
            candidates.insert(redefReg->getParent());
        }
      }
    }
  }

  // handle successor blocks.
  if (itr == end && std::distance(mbb->succ_begin(), mbb->succ_end()) > 0) {
    std::for_each(mbb->succ_begin(), mbb->succ_end(), [&](MachineBasicBlock *succ) {
      return getCandidateInsertionPositionsDFS(succ->begin(), candidates, visited);
    });
  }
}

void RegisterRenaming::getCandidatePosForBoundaryInsert(MachineInstr *startPos,
                                                        std::set<MachineInstr *> &candidates,
                                                        std::set<MachineBasicBlock *> &visited) {
  assert(startPos && startPos->getParent());
  candidates.insert(startPos);

  getCandidateInsertionPositionsDFS(startPos, candidates, visited);
}

void RegisterRenaming::computeDistance(ItrRange range,
                                       std::set<MachineInstr *> &candidates,
                                       std::map<MachineInstr *, unsigned> &dists,
                                       unsigned distance,
                                       std::set<MachineBasicBlock *> &visited) {
  std::vector<ItrRange> worklsit;
  worklsit.push_back(range);

  while (!worklsit.empty()) {
    auto r = worklsit.back();
    worklsit.pop_back();
    if (r.first == r.second)
      continue;
    auto mi = r.first;
    assert(mi->getParent());
    auto mbbEnd = r.second;
    auto mbb = mi->getParent();

    if (std::distance(mi, mbbEnd) >= 0) {

      if (!visited.insert(mbb).second)
        continue;
      IDEM_DEBUG(mi->dump(););

      for (; mi != mbbEnd && !tii->isIdemBoundary(&*mi); ++mi, ++distance) {
        if (!candidates.count(&*mi))
          continue;

        dists[&*mi] = distance;
      }
    }

    std::for_each(mbb->pred_begin(), mbb->pred_end(), [&](MachineBasicBlock *pred) {
      worklsit.push_back({pred->rbegin(), pred->rend()});
    });
  }
}

void RegisterRenaming::computeOptimizedInsertion(MachineInstr *startPos,
                                                 std::set<MachineInstr *> &candidates,
                                                 std::vector<MachineInstr *> &InsertedPos) {
  assert(startPos && startPos->getParent());

  auto itr = candidates.begin();

  // removes those machine instr after startPos. Because needed candidate should
  // be prior to startPos, so just ignore it!
  //
  // Note that, we should skip the startPos instruction.
  for (; itr != candidates.end(); ++itr) {
    if (*itr != startPos && (llvm::reachable(startPos, *itr) ||
        !llvm::reachable(*itr, startPos))) {
      candidates.erase(itr);
    }
  }

  // If there is no other insertion position, current use MI is suitable.
  candidates.insert(startPos);

  // As we reach here, all MI in candidates should encounter startPos when
  // advance forward along with CFG edge.
  std::map<MachineInstr *, unsigned> dists;
  std::set<MachineBasicBlock *> visited;
  ItrRange range = {--MachineBasicBlock::reverse_iterator(startPos), startPos->getParent()->rend()};
  computeDistance(range, candidates, dists, 0, visited);

  std::vector<std::pair<MachineInstr *, unsigned> > tmp;
  for (std::pair<MachineInstr *, unsigned> pair : dists) {
    tmp.emplace_back(pair.first, pair.second);
  }

  std::sort(tmp.begin(), tmp.end(), [](std::pair<MachineInstr *, unsigned> o1,
                                       std::pair<MachineInstr *, unsigned> o2) { return o1.second > o2.second; });

  unsigned max = 0;
  for (std::pair<MachineInstr *, unsigned> pair : tmp) {
    if (InsertedPos.empty()) {
      InsertedPos.push_back(pair.first);
      max = pair.second;
    } else if (pair.second == max)
      InsertedPos.push_back(pair.first);
    else
      break;
  }
}

bool RegisterRenaming::handleMultiDepsWithinSameMI(AntiDepPair &pair) {
  if (pair.use->getParent() != pair.def->getParent())
    return false;

  auto mi = pair.use->getParent();
  std::set<MachineOperand *> defs;
  std::set<MachineOperand *> uses;
  getDefUses(mi, &defs, &uses, allocaSet);

  // Collects all anti-dependence within the same mi.
  std::vector<AntiDepPair> list;
  for (auto &defMO : defs) {
    for (auto &useMO : uses) {
      if (predEq(defMO, useMO) && !contain(prevDefRegs[useMO->getParent()], useMO, predEq)) {
        list.emplace_back(useMO, defMO);
        goto BREAK;
      }
    }
    BREAK:
    continue;
  }
  // If there is not multiple anti-dependences within the same mi, use other function
  // to handle this case.
  if (list.size() <= 1)
    return false;

  for (auto elt : list) {
    auto res = contains(antiDeps.begin(), antiDeps.end(), elt);
    if (res.second)
      antiDeps.erase(res.first);
  }
  for (auto elt : list) {
    auto res = contains(antiDeps.begin(), antiDeps.end(), elt);
    if (res.second)
      antiDeps.erase(res.first);
  }

  // Inserts some move instr right before use MI and surrounds it with splitting boundary.
  assert(list.size() <= NUM - 2 && "Can't handle too many anti-dependences within the same MI!");


  // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
  std::set<MachineInstr *> candidateInsertPos;
  std::set<MachineBasicBlock *> visited;
  getCandidatePosForBoundaryInsert(mi, candidateInsertPos, visited);

  std::vector<MachineInstr *> optInsertedPos;
  computeOptimizedInsertion(mi, candidateInsertPos, optInsertedPos);

  for (auto pos : optInsertedPos) {
    auto mbb = pos->getParent();

    // Inserts two boundary instruction to surround the move instr.
    tii->emitIdemBoundary(*mbb, pos);
    tii->emitIdemBoundary(*mbb, pos);
    auto boundary = getPrevMI(pos);

    // the end index of last copy instr to be inserted.
    unsigned to = li->mi2Idx[pos];
    for (auto &pair : list) {
      LiveIntervalIdem *interval = new LiveIntervalIdem;
      auto from = to - 1;

      interval->addRange(from, to);    // add an interval for a temporal move instr.
      unsigned phyReg = choosePhysRegForRenaming(pair.use, interval);

      assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
      assert(phyReg != pair.use->getReg());

      // Step#8: substitute the old reg with phyReg,
      // and remove other anti-dep on this use.
      unsigned oldReg = pair.use->getReg();
      //clearAntiDeps(pair.use);
      pair.use->setReg(phyReg);

      if (!pair.usesInSameMI.empty())
        std::for_each(pair.usesInSameMI.begin(), pair.usesInSameMI.end(),
                      [=](MachineOperand *p) { p->setReg(phyReg); });

      assert(tii->isIdemBoundary(boundary) && "the mi at inserted position must be a splitting boundary!");
      // Step#10: insert a move instruction before splitting boundary instr.
      // This instruction would be the last killer of src reg in this copy instr.
      tii->copyPhysReg(*mbb, boundary, DebugLoc(), phyReg, oldReg, true);

      // annotate the undef flag to the src reg if src reg is liveIn.
      auto copyMI = getPrevMI(boundary);

      // attach the undef flag to all src regs.
      for (unsigned i = 1, e = copyMI->getNumOperands(); i < e; ++i) {
        auto &mo = copyMI->getOperand(i);
        if (mo.isReg() && mo.getReg())
          mo.setIsUndef(true);
      }
      // Update prev defs and uses dataflow.
      updatePrevDefUses();
    }
  }

  // remove those anti-dependence pairs from antiDeps list.
  for (auto pair : list) {
    auto pos = std::find(antiDeps.begin(), antiDeps.end(), pair);
    if (pos != antiDeps.end())
      antiDeps.erase(pos);
  }
  return true;
}

bool RegisterRenaming::idemCanBeRemoved(MachineInstr *mi) {
  assert(tii->isIdemBoundary(mi) && "Only allow to entry this function when mi is idem!");
  const MachineBasicBlock &entryMBB = mf->front();
  if (mi == &entryMBB.front())
    return true;

  // Re-compute the prevDef and prevUse set for each instruction after mi.
  // copy
  std::map<MachineInstr *, std::set<MachineOperand *>> localPrevDefs = prevDefRegs;
  std::map<MachineInstr *, std::set<MachineOperand *>> localPrevUses = prevUseRegs;

  MachineBasicBlock::iterator itr = mi;
  MachineBasicBlock::iterator end = mi->getParent()->end();

  bool removable = true;
  bool firstIdem = true;
  for (; itr != end && (firstIdem || !tii->isIdemBoundary(itr)); ++itr) {
    firstIdem = false;

    std::set<MachineOperand *> defs;
    std::set<MachineOperand *> uses;
    getDefUses(itr, &defs, &uses, allocaSet);

    SmallVectorImpl<IdempotentRegion *> Regions(10);
    regions->getRegionsContaining(*itr, &Regions);
    computeDefUseDataflow<true>(itr, uses, &Regions, localPrevDefs, localPrevUses);

    // checks
    auto prevDefs = localPrevDefs[itr];
    if (prevDefs.empty())
      continue;

    auto prevUses = localPrevUses[itr];
    for (auto defMO : defs) {
      for (MachineOperand *useMO : prevUses) {
        assert(useMO->isReg());
        if (useMO->getReg() != defMO->getReg())
          continue;

        // We don't need to check whether is the useMI in the same region as
        // defMI, because our algorithm ensures useMI and defIMI must are in
        // the separate regions.
        if (!contain(localPrevDefs[useMO->getParent()], useMO, predEq)) {
          removable = false;
          goto RETURN;
        }
      }
    }
  }

  // If the idem instr can be erased, so update the global prevUses and prevDefs
  // set.
  prevUseRegs = localPrevUses;
  prevDefRegs = localPrevDefs;

  RETURN:
  return removable;
}

bool clearUselessIdem(std::vector<MachineInstr *> &removable) {
  bool changed = false;
  if (!removable.empty()) {
    std::for_each(removable.begin(), removable.end(), [&](MachineInstr *mi) {
      mi->eraseFromParent();
      changed = true;
    });
    removable.clear();
  }
  return changed;
}

bool RegisterRenaming::scavengerIdem() {

  std::vector<MachineInstr *> removable;

  bool changed = false;
  for (auto &mbb : *mf) {
    if (mbb.empty())
      continue;

    // In simply case, delete a sequence of idem call instr.
    auto itr = mbb.begin();
    for (; itr != mbb.end();) {

      if (!tii->isIdemBoundary(&*itr)) {
        ++itr;
        continue;
      }

      auto next = ++itr;
      while (next != mbb.end() && tii->isIdemBoundary(&*next)) {
        removable.push_back(const_cast<MachineInstr *>(&*next++));
      }
      itr = next;
    }
    changed |= clearUselessIdem(removable);

    itr = mbb.begin();
    for (; itr != mbb.end(); ++itr) {
      auto mi = const_cast<MachineInstr *>(&*itr);
      if (!tii->isIdemBoundary(mi))
        continue;

      if (idemCanBeRemoved(mi)) {
        // remove this mi.
        removable.push_back(mi);
      }
    }

    changed |= clearUselessIdem(removable);
  }
  return changed;
}

typedef std::vector<std::vector<AntiDepPair>> SingleDefMultiUses;

/**
 * Checks if it is profitable to merge the second anti-dep into first one
 * based on following criteria:
 * second's def == first's def && second's use != first's use &&
 * second's use's reg == first'use'reg
 *
 * @param first
 * @param second
 * @return  Return true if the checked condition is satisfied.
 */
static bool isProfitableToMerge(AntiDepPair &first, AntiDepPair& second) {
  assert(first.use->getParent() && first.def->getParent());
  assert(second.use->getParent() && second.def->getParent());

  return second.def == first.def && second.use != first.use &&
  second.use->getReg() == first.use->getReg();
}

AntiDepPair* findForemostUse(std::vector<AntiDepPair> &list, LiveIntervalAnalysisIdem *li) {
  if (list.empty()) return nullptr;

  MachineBasicBlock *mbb = 0;
  for (AntiDepPair &pair : list) {
    if (mbb == 0)
      mbb = pair.use->getParent()->getParent();
    else if (mbb != pair.use->getParent()->getParent()) {
      mbb = 0;
      break;
    }
  }
  // All uses are in the same machine basic block.
  if (mbb) {
    unsigned minIndex = UINT32_MAX;
    AntiDepPair *res = 0;
    for (size_t i = 0, e = list.size(); i < e; i++) {
      auto idx = li->mi2Idx[list[i].use->getParent()];
      if (minIndex > idx) {
        minIndex = idx;
        res = &list[i];
      }
    }
    assert(minIndex != UINT32_MAX && res);
    return res;
  }

  return nullptr;
  // all uses are in different basic block.
  // currently, we don't handle this case

  std::map<MachineBasicBlock*, std::pair<unsigned, MachineOperand*>> minIndexForMBBs;
  for (AntiDepPair &pair : list) {
    assert(pair.use->getParent());
    auto mbb = pair.use->getParent()->getParent();
    auto idx = li->mi2Idx[pair.use->getParent()];
    assert(mbb);

    if (!minIndexForMBBs.count(mbb) || minIndexForMBBs[mbb].first > idx) {
      minIndexForMBBs[mbb] = std::make_pair(idx, pair.use);
    }
  }
}

bool RegisterRenaming::handleSingleDefMultiUses() {
  SingleDefMultiUses worklist;
  for (auto &pair : antiDeps) {
    auto tmp = std::vector<AntiDepPair>();
    tmp.push_back(pair);
    worklist.push_back(tmp);
  }
  // merge the pairs in worklist.
  bool changed = true;
  do {
    changed = false;
    for (size_t i = 0; i < worklist.size(); i++) {
      for (size_t j = i + 1; j < worklist.size(); j++)
        if (isProfitableToMerge(worklist[i].front(), worklist[j].front())) {
          worklist[i].insert(worklist[i].end(), worklist[j].begin(), worklist[j].end());
          worklist[j] = worklist.back();
          worklist.pop_back();
          --j;
          changed = true;
        }
    }
  }while (changed);


  changed = false;
  while (!worklist.empty()) {
    // find the foremost use
    auto temp = worklist.back();
    worklist.pop_back();
    if (temp.size() < 2)
      continue;

    IDEM_DEBUG(llvm::errs()<<"\n Merged multiples anti-deps:\n";
    for (auto &pair : temp) {
      llvm::errs()<<"["<<li->mi2Idx[pair.def->getParent()] << ", " <<
                    li->mi2Idx[pair.use->getParent()] << ", " <<
                    tri->getName(pair.use->getReg()) << "]\n";
    });

    if (AntiDepPair *insertPos = findForemostUse(temp, li)) {
      std::vector<MachineOperand*> uses;
      for (auto &pair : temp) {
        if (&pair != insertPos)
          uses.push_back(pair.use);
      }
      insertMoveAndBoundary(*insertPos, &uses);
      // renaming other use on the same register
      // remove the group anti-deps pair from antiDeps list.
      for (auto &pair : temp) {
        auto pos = std::find(antiDeps.begin(), antiDeps.end(), pair);
        if(pos != antiDeps.end())
          antiDeps.erase(pos);
      }

      changed = true;
    }

  }
  return changed;
}

bool RegisterRenaming::runOnMachineFunction(MachineFunction &MF) {
  mf = &MF;
  tii = MF.getTarget().getInstrInfo();
  tri = MF.getTarget().getRegisterInfo();
  li = getAnalysisIfAvailable<LiveIntervalAnalysisIdem>();
  mfi = MF.getFrameInfo();
  mri = &MF.getRegInfo();
  td = MF.getTarget().getTargetData();
  allocaSet = tri->getAllocatableSet(*mf);

  // Step#1: Collects regions
  regions = getAnalysisIfAvailable<MachineIdempotentRegions>();
  dt = getAnalysisIfAvailable<MachineDominatorTree>();
  assert(dt);

  // We must ensure that we will not change the CFG of this Function.
  // The only thing we need to modify is inserting boundary instr as
  // appropriate.

  computeReversePostOrder(MF, *dt, sequence);
  bool changed = false;

  //llvm::errs()<<"Deal with: "<<MF.getFunction()->getName()<<"\n";
  do {
    reconstructIdemAndLiveInterval();
    // Step#2: visits register operand of each machine instr in the program sequence.
    for (auto &mbb : sequence) {
      auto mi = mbb->instr_begin();
      auto mie = mbb->instr_end();
      for (; mi != mie; ++mi) {
        assert(li->mi2Idx.count(mi));

        // Step#3: collects reg definition information.
        // Step#4: collects reg uses information.
        SmallVectorImpl<IdempotentRegion *> Regions(10);
        regions->getRegionsContaining(*mi, &Regions);
        collectRefDefUseInfo(mi, &Regions);
      }
    }

    // If there is not antiDeps exist, just early break from do loop.
    if (antiDeps.empty())
      break;

    // Step#5: checks if we should rename the defined register according to usd-def,
    //         If it is, construct a pair of use-def reg pair.
    simplifyAntiDeps();

    // IDEM_DEBUG(llvm::errs()<<"\n**************** Round #"<<round<<" ***************\n"; dump(););
    IDEM_DEBUG(
        llvm::errs() << "\n************* All Anti-dependences *************\n";
        for (auto pair : antiDeps) {
          unsigned defIdx = li->mi2Idx[pair.def->getParent()];
          llvm::errs() << "[" << defIdx << ',' << li->mi2Idx[pair.use->getParent()] <<
                       ", " << tri->getName(pair.use->getReg()) << "]\n";
          for (auto use : pair.usesInSameMI)
            llvm::errs() << "[" << defIdx << ',' << li->mi2Idx[use->getParent()] <<
                         ", " << tri->getName(use->getReg()) << "]\n";
        });

    // Before deal with other kinds of anti-dependence pair, we
    // attempt to single-def and multiple-uses on the same register.
    changed |= handleSingleDefMultiUses();

    while (!antiDeps.empty()) {
      auto pair = antiDeps.front();
      antiDeps.pop_front();

      if (shouldRename(pair)) {
        // Step#6: choose a physical register for renaming.
        // Step#7: if there is no free physical register, using heuristic method to
        //         spill out a interval.
        if (!handleMultiDepsWithinSameMI(pair))
          insertMoveAndBoundary(pair);

        changed = true;
      }
    }

    // FIXME, cleanup is needed for transforming some incorrect code into normal status.
    bool localChanged;
    do {
      localChanged = scavengerIdem();
      changed |= localChanged;
    } while (localChanged);

    changed |= localChanged;
  }while (true);

  // If we are not going to clear the antiDeps, there is an item
  // remained produced by previous running of this pass.
  // I don't know why???
  releaseMemory();

  return changed;
}

