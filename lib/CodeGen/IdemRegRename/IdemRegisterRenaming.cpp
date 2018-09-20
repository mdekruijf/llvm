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
#include <llvm/Support/Debug.h>
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
    std::vector<MachineOperand*> usesInSameMI;

    AntiDepPair(MachineOperand *_use, MachineOperand *_def) : use(_use), def(_def) {}

    bool operator == (AntiDepPair rhs) {
      return use == rhs.use && def == rhs.def;
    }

    bool operator != (AntiDepPair rhs) {
      return !(*this == rhs);
    }

    bool operator < (const AntiDepPair &rhs) {
      if ((void*)use < (void*)rhs.use) return true;
      else return (void*)def < (void*)rhs.def;
    }

    bool operator < (const AntiDepPair &rhs) const {
      return *const_cast<AntiDepPair*>(this) < rhs;
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
    template<bool IgnoreIdem = false>
    void computeDefUseDataflow(MachineInstr *mi,
                               std::set<MachineOperand*> &uses,
                               SmallVectorImpl<IdempotentRegion *> *Regions,
                               std::map<MachineInstr*, std::set<MachineOperand*>> &prevDefs,
                               std::map<MachineInstr*, std::set<MachineOperand*>> &prevUses);
    inline void addAntiDeps(MachineOperand *useMO, MachineOperand *defMO);
    void collectRefDefUseInfo(MachineInstr *mi, SmallVectorImpl<IdempotentRegion *> *Regions);
    bool shouldRename(AntiDepPair pair);
    void filterUnavailableRegs(MachineOperand* use, BitVector &allocSet);
    unsigned choosePhysRegForRenaming(MachineOperand *use,
                                      LiveIntervalIdem *interval);
    unsigned tryChooseFreeRegister(LiveIntervalIdem &interval,
                                   const TargetRegisterClass &rc,
                                   BitVector &allocSet);
    bool availableOnRegClass(unsigned physReg, const TargetRegisterClass &rc);

    unsigned tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                      const TargetRegisterClass &rc,
                                      BitVector &allocSet);

    void spillOutInterval(LiveIntervalIdem *interval);

    void simplifyAntiDeps();

    void clearAntiDeps(MachineOperand *useMO);

    void updatePrevDefUses();

    void insertMoveAndBoundary(AntiDepPair &pair);

    void getCandidateInsertionPositionsDFS(MachineInstr *startPos,
                                           std::set<MachineInstr *> &candidates,
                                           std::set<MachineBasicBlock *> &visited);

    void getCandidatePosForBoundaryInsert(MachineInstr *startPos,
                                          std::set<MachineInstr*> &candidates,
                                          std::set<MachineBasicBlock*> &visited);

    void computeDistance(MachineInstr *startPos,
                         std::set<MachineInstr*> &candidates,
                         std::map<MachineInstr*, unsigned> &dists,
                         unsigned distance,
                         std::set<MachineBasicBlock*> &visited);

    void computeOptimizedInsertion(MachineInstr *startPos,
                                            std::set<MachineInstr*> &candidates,
                                            std::vector<MachineInstr*> &InsertedPos);

    bool handleMultiDepsWithinSameMI(AntiDepPair &pair);
    bool handleMultiUseOneDefWithinSameMI(AntiDepPair &pair);

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

    // records all def registers by instr before current mi.
    std::map<MachineInstr*, std::set<MachineOperand*>> prevDefRegs;
    // records all use registers of current mi and previous mi.
    std::map<MachineInstr*, std::set<MachineOperand*>> prevUseRegs;
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

FunctionPass* llvm::createRegisterRenamingPass() {
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

template <class T>
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

template <class T>
static void intersect(std::set<T> &res, std::set<T> lhs, std::set<T> rhs) {
  for (T elt : lhs) {
    if (!rhs.count(elt))
      res.insert(elt);
  }
}

struct DefUseOfMI {
  std::set<MachineOperand*> *uses;
  std::set<unsigned> *defs;
};

std::map<MachineInstr*, DefUseOfMI*> map;

DefUseOfMI* getOrCreateDefUse(MachineInstr *mi,
                              std::set<MachineOperand*> *uses,
                              std::set<unsigned> *defs) {
  DefUseOfMI *&res = map[mi];
  if (!res)
    res = new DefUseOfMI{uses, defs};
  return res;
}

static void getDefUses(MachineInstr *mi,
                       std::set<MachineOperand*> *defs,
                       std::set<MachineOperand*> *uses,
                       const BitVector &allocaSets) {
  for (unsigned i = 0, e = mi->getNumOperands(); i < e; i++) {
    MachineOperand *mo = &mi->getOperand(i);
    if (!mo || !mo->isReg() ||
        !mo->getReg() || mo->isImplicit() ||
        !allocaSets.test(mo->getReg())) continue;

    unsigned &&reg = mo->getReg();
    assert(TargetRegisterInfo::isPhysicalRegister(reg));

    if (mo->isDef() && defs) {
      defs->insert(mo);
    }
    else if (mo->isUse() && uses)
      uses->insert(mo);
  }
}

bool contains(IdempotentRegion::inst_iterator begin,
              const IdempotentRegion::inst_iterator &end,
              MachineInstr *elt) {
  auto itr = std::move(begin);
  while(itr != end) {
    if (*itr == elt)
      return true;
    ++itr;
  }
  return false;
}

MachineInstr *getPrevMI(MachineInstr *mi) {
  if (!mi || !mi->getParent()) return nullptr;
  return ilist_traits<MachineInstr>::getPrev(mi);
}

MachineInstr *getNextMI(MachineInstr *mi) {
  if (!mi || !mi->getParent()) return nullptr;
  return ilist_traits<MachineInstr>::getNext(mi);
}

bool regionContains(IdempotentRegion *region, MachineInstr *mi) {
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

bool regionContains(SmallVectorImpl<IdempotentRegion *> *Regions, MachineInstr *mi) {
  for (IdempotentRegion *r : *Regions) {
    if (regionContains(r, mi))
      return true;
  }
  return false;
}


inline static bool predEq(const MachineOperand *o1, const MachineOperand *o2)
{ return o1->getReg() == o2->getReg(); }


template<typename T, typename _InputIterator>
std::pair<_InputIterator, bool> contains(_InputIterator first, _InputIterator last, T t) {

  for (; first != last; ++first) {
    if (*first == t)
      return std::pair<_InputIterator, bool>(first, true);
  }
  return std::pair<_InputIterator, bool>(_InputIterator(), false);
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
                                             std::set<MachineOperand*> &uses,
                                             SmallVectorImpl<IdempotentRegion *> *Regions,
                                             std::map<MachineInstr*, std::set<MachineOperand*>> &prevDefs,
                                             std::map<MachineInstr*, std::set<MachineOperand*>> &prevUses) {
  if (regions->isRegionEntry(*mi) && !IgnoreIdem) {
    prevDefs[mi] = std::set<MachineOperand*>();
    prevUses[mi] = uses;
  }
  else {
    // if the mi is the first mi of basic block with preds.
    if (mi == &mi->getParent()->front()) {
      MachineBasicBlock *mbb = mi->getParent();
      if (mbb->pred_empty()) {
        prevDefs[mi] = std::set<MachineOperand*>();
        prevUses[mi] = uses;
      }
      else {
        std::set<MachineOperand*> &predDefs = prevDefs[mi];
        std::set<MachineOperand*> &predUses = prevUses[mi];

        auto itr = mbb->pred_begin();
        auto end = mbb->pred_end();
        for (; itr != end; ++itr) {
          MachineBasicBlock *pred = *itr;
          if (pred->empty())
            continue;

          MachineInstr *predMI = &pred->back();
          if (!regionContains(Regions, predMI))
            continue;

          std::set<MachineOperand*> localUses;
          std::set<MachineOperand*> localDefs;

          getDefUses(predMI, &localDefs, &localUses, allocaSet);

          predDefs = Union(localDefs, prevDefs[predMI], predEq);
          predUses = Union(localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    }
    else {
      // otherwise
      std::set<MachineOperand*> localPrevDefs;
      MachineInstr *prevMI = getPrevMI(mi);
      DEBUG(prevMI->dump());

      assert(prevMI && "previous machine instr can't be null!");
      getDefUses(prevMI, &localPrevDefs, 0, allocaSet);

      prevDefs[mi] = Union(prevDefs[prevMI], localPrevDefs, predEq);
      prevUses[mi] = Union(prevUses[prevMI], uses);

      DEBUG(for (auto def : prevDefs[mi])
        llvm::errs()<<tri->getName(def->getReg())<<" ";
      llvm::errs()<<"\n";
      for (auto use : prevUses[mi])
        llvm::errs()<<tri->getName(use->getReg())<<" ";
      llvm::errs()<<"\n";);
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
  if (!mi) return;
  std::set<MachineOperand*> uses;
  std::set<MachineOperand*> defs;
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

bool RegisterRenaming::shouldRename(AntiDepPair pair) {
  auto use = pair.use;
  MachineInstr *useMI = use->getParent();
  std::set<MachineOperand*> &defs = prevDefRegs[useMI];
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
      getNextMI(mi)->getOperand(0).setIsUndef(true);
    }
    else if (mo->isUse()) {
      assert(frameIndex != INT_MIN);
      tii->loadRegFromStackSlot(*mi->getParent(), mi, mo->getReg(), frameIndex, rc, tri);
      // Inserts a boundary instruction immediately before the load to partition the
      // region into two different parts for avoiding violating idempotence.
      auto ld = getPrevMI(mi);
      tii->emitIdemBoundary(*mi->getParent(), ld);
    }
  }
}

bool RegisterRenaming::availableOnRegClass(unsigned physReg,
                                           const TargetRegisterClass &rc) {
  return tri->getMinimalPhysRegClass(physReg)->hasSubClassEq(&rc) /*&& !mri->isLiveIn(physReg)*/;
}

unsigned RegisterRenaming::tryChooseFreeRegister(LiveIntervalIdem &interval,
                                                 const TargetRegisterClass &rc,
                                                 BitVector &allocSet) {
  DEBUG(llvm::errs()<<"Interval for move instr: ";
  interval.dump(*const_cast<TargetRegisterInfo*>(tri)); llvm::errs()<<"\n";);

  for (int physReg = allocSet.find_first(); physReg > 0; physReg = allocSet.find_next(physReg)) {
    if (li->intervals.count(physReg)) {
      LiveIntervalIdem *itrv = li->intervals[physReg];

      DEBUG(llvm::errs()<<"Candidated interval: ";
      itrv->dump(*const_cast<TargetRegisterInfo*>(tri));
      llvm::errs()<<"\n";);

      if (!itrv->intersects(&interval)) {
        // we only consider those live interval which doesn't interfere with current
        // interval.
        // No matching in register class should be ignored.
        // Avoiding LiveIn register(such as argument register).
        if (!availableOnRegClass(physReg, rc))
          continue;

        return physReg;
      }
    }
    else if (availableOnRegClass(physReg, rc)){
      // current physReg is free, so return it.
      return static_cast<unsigned int>(physReg);
    }
  }
  return 0;
}

unsigned RegisterRenaming::tryChooseBlockedRegister(LiveIntervalIdem &interval,
                                                    const TargetRegisterClass &rc,
                                                    BitVector &allocSet) {
  // choose an interval to be evicted out memory, and insert spilling code as
  // appropriate.
  unsigned costMax = INT_MAX;
  LiveIntervalIdem *targetInter = nullptr;
  for (auto physReg = allocSet.find_first(); physReg > 0;
            physReg = allocSet.find_next(physReg)) {
    if(!li->intervals.count(physReg))
      continue;
    auto phyItv = li->intervals[physReg];

    // TargetRegisterClass isn't compatible with each other.
    if (!availableOnRegClass(phyItv->reg, rc))
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
  if (!targetInter) return 0;

  DEBUG(llvm::errs()<<"Selected evicted physical register is: "
  << tri->getName(targetInter->reg)<<"\n";
  llvm::errs()<<"\nSelected evicted interval is: ";
  targetInter->dump(*const_cast<TargetRegisterInfo*>(tri)););

  spillOutInterval(targetInter);
  return targetInter->reg;
}

void RegisterRenaming::filterUnavailableRegs(MachineOperand* use, BitVector &allocSet) {

  DEBUG(for (int i = allocSet.find_first(); i > 0; i = allocSet.find_next(i)) {
    llvm::errs()<<tri->getName(i)<<" ";
  }
            llvm::errs()<<"\n";);

  // remove the defined register by use mi from allocable set.
  std::set<MachineOperand*> defs;
  getDefUses(use->getParent(), &defs, 0, tri->getAllocatableSet(*mf));
  for (MachineOperand* phy : defs)
    allocSet[phy->getReg()] = false;

  // also, we must make sure no the physical register same as
  // use will be assigned.
  allocSet[use->getReg()] = false;

  std::vector<MachineInstr*> worklist;
  worklist.push_back(use->getParent());
  std::set<MachineBasicBlock*> visited;

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

unsigned RegisterRenaming::choosePhysRegForRenaming(MachineOperand *use,
                                                    LiveIntervalIdem *interval) {
  auto rc = tri->getMinimalPhysRegClass(use->getReg());
  auto allocSet = tri->getAllocatableSet(*mf);

  // Remove some registers are not avaiable when making decision of choosing.
  filterUnavailableRegs(use, allocSet);

  // obtains a free register used for move instr.
  unsigned freeReg = tryChooseFreeRegister(*interval, *rc, allocSet);
  if (!freeReg) {
    freeReg = tryChooseBlockedRegister(*interval, *rc, allocSet);
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
        antiDeps[j] = antiDeps[e-1];
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
        antiDeps[j] = antiDeps[e-1];
        antiDeps.pop_back();
        --j;
        --e;
      }
    }
  }
}

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
    }
    else
      ++itr;
  }
}

void RegisterRenaming::updatePrevDefUses() {
  // FIXME, 9/17/2018, we need update prevDef, prevUses reg set, and idempotence regions.
  regions->releaseMemory();
  regions->runOnMachineFunction(*const_cast<MachineFunction*>(mf));

  for (auto &mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto mie = mbb->instr_end();
    for (; mi != mie; ++mi) {
      // Step#3: collects reg definition information.
      // Step#4: collects reg uses information.
      SmallVectorImpl<IdempotentRegion *> Regions(10);
      regions->getRegionsContaining(*mi, &Regions);

      std::set<MachineOperand*> uses;
      std::set<MachineOperand*> defs;
      getDefUses(mi, &defs, &uses, allocaSet);

      computeDefUseDataflow(&*mi, uses, &Regions, prevDefRegs, prevUseRegs);
    }
  }
}

void RegisterRenaming::insertMoveAndBoundary(AntiDepPair &pair) {
  auto useMI = pair.use->getParent();

  // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
  //insertBoundaryAsNeed(useMI, phyReg);
  MachineBasicBlock *mbb = 0;

  // Find the candiate insertion positions.
  std::set<MachineInstr*> candidateInsertPos;
  std::set<MachineBasicBlock*> visited;
  getCandidatePosForBoundaryInsert(useMI, candidateInsertPos, visited);

  DEBUG(llvm::errs()<<"candiates insertion positions:\n";
  for(auto mi : candidateInsertPos) mi->dump(););

  // Computes the optimizing positions.
  std::vector<MachineInstr *> optInsertedPos;
  computeOptimizedInsertion(useMI, candidateInsertPos, optInsertedPos);

  // insert a boundary right after the use MI for avoiding validating the idem region
  // after use MI.

  // Inserts a poir of boundary and move instrs at each insertion point.
  for (auto pos : optInsertedPos) {
    assert(pos && pos->getParent() &&
    pos->getParent()->getParent() == mf);

    mbb = pos->getParent();

    // FIXME, 9/17/2018. Now, live range computes correctly .
    LiveIntervalIdem *interval = new LiveIntervalIdem;
    auto to = li->getIndex(useMI);
    auto from = li->getIndex(pos) - 2;

    interval->addRange(from, to);    // add an interval for a temporal move instr.
    unsigned phyReg = choosePhysRegForRenaming(pair.use, interval);

    assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
    assert(phyReg != pair.use->getReg());

    // Step#8: substitute the old reg with phyReg,
    // and remove other anti-dep on this use.
    unsigned oldReg = pair.use->getReg();
    clearAntiDeps(pair.use);

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
  }
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
    std::set<MachineOperand*> defs;
    SmallVectorImpl<IdempotentRegion *> Regions(20);
    regions->getRegionsContaining(*itr, &Regions);

    getDefUses(itr, &defs, 0, allocaSet);
    auto prevUses = prevUseRegs[itr];
    for (auto defMO : defs) {
      for (MachineOperand* useMO : prevUses) {
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
    std::for_each(mbb->succ_begin(), mbb->succ_end(), [&](MachineBasicBlock *succ)
    {
      return getCandidateInsertionPositionsDFS(succ->begin(), candidates, visited);
    });
  }
}

void RegisterRenaming::getCandidatePosForBoundaryInsert(MachineInstr *startPos,
                                                        std::set<MachineInstr*> &candidates,
                                                        std::set<MachineBasicBlock*> &visited) {
  assert(startPos && startPos->getParent());
  candidates.insert(startPos);

  getCandidateInsertionPositionsDFS(startPos, candidates, visited);
}

void RegisterRenaming::computeDistance(MachineInstr *startPos,
                     std::set<MachineInstr*> &candidates,
                     std::map<MachineInstr*, unsigned> &dists,
                     unsigned distance,
                     std::set<MachineBasicBlock*> &visited) {
  assert(startPos && startPos->getParent());
  auto mbb = startPos->getParent();

  if (!visited.insert(mbb).second)
    return;

  auto mbbEnd = mbb->rend();
  MachineBasicBlock::reverse_iterator mi(startPos);
  --mi;

  for (; mi != mbbEnd && !tii->isIdemBoundary(&*mi); ++mi, ++distance) {
    if (!candidates.count(&*mi))
      continue;

    dists[&*mi] = distance;
  }

  if (mi == mbbEnd) {
    std::for_each(mbb->pred_begin(), mbb->pred_end(), [&](MachineBasicBlock *pred)
    { return computeDistance(&*pred->rbegin(), candidates, dists, distance, visited); });
  }
}


void RegisterRenaming::computeOptimizedInsertion(MachineInstr *startPos,
                               std::set<MachineInstr*> &candidates,
                               std::vector<MachineInstr*> &InsertedPos) {
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
  std::map<MachineInstr*, unsigned> dists;
  std::set<MachineBasicBlock*> visited;
  computeDistance(startPos, candidates, dists, 0, visited);

  std::vector<std::pair<MachineInstr*, unsigned> > tmp;
  for (std::pair<MachineInstr*, unsigned> pair : dists) {
    tmp.emplace_back(pair.first, pair.second);
  }

  std::sort(tmp.begin(), tmp.end(), [](std::pair<MachineInstr*, unsigned> o1,
      std::pair<MachineInstr*, unsigned> o2) { return o1.second > o2.second; });

  unsigned max = 0;
  for (std::pair<MachineInstr*, unsigned> pair : tmp) {
    if (InsertedPos.empty()) {
      InsertedPos.push_back(pair.first);
      max = pair.second;
    }
    else if (pair.second == max)
      InsertedPos.push_back(pair.first);
    else
      break;
  }
}

bool RegisterRenaming::handleMultiDepsWithinSameMI(AntiDepPair &pair) {
  if (pair.use->getParent() != pair.def->getParent())
    return false;

  auto mi = pair.use->getParent();
  std::set<MachineOperand*> defs;
  std::set<MachineOperand*> uses;
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
    BREAK: continue;
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
  std::set<MachineInstr*> candidateInsertPos;
  std::set<MachineBasicBlock*> visited;
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
      clearAntiDeps(pair.use);
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
  std::map<MachineInstr*, std::set<MachineOperand*>> localPrevDefs = prevDefRegs;
  std::map<MachineInstr*, std::set<MachineOperand*>> localPrevUses = prevUseRegs;

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

bool RegisterRenaming::scavengerIdem() {

  std::vector<MachineInstr*> removable;

  bool  changed = false;
  for (auto &mbb : *mf) {
    if (mbb.empty())
      continue;

    auto itr = mbb.begin();
    for (; itr != mbb.end(); ++itr) {
      auto mi = const_cast<MachineInstr*>(&*itr);
      if (!tii->isIdemBoundary(mi))
        continue;

      if (idemCanBeRemoved(mi)) {
        // remove this mi.
        removable.push_back(mi);
        /*
        mi->eraseFromParent();
        regions->releaseMemory();
        regions->runOnMachineFunction(*const_cast<MachineFunction*>(mf));
        for (auto &MBB : sequence) {
          auto MI = MBB->instr_begin();
          auto MIE = MBB->instr_end();
          for (; MI != MIE; ++MI) {
            // Step#3: collects reg definition information.
            // Step#4: collects reg uses information.
            SmallVectorImpl<IdempotentRegion *> Regions(10);
            regions->getRegionsContaining(*MI, &Regions);
            collectRefDefUseInfo(MI, &Regions);
          }
        }
         */
      }
    }
  }

  if (!removable.empty()) {
    changed = true;
    for (MachineInstr* mi : removable)
      mi->eraseFromParent();
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

  // If we are not going to clear the antiDeps, there is an item
  // remained produced by previous running of this pass.
  // I don't know why???
  antiDeps.clear();

  dt = getAnalysisIfAvailable<MachineDominatorTree>();
  assert(dt);

  // We must ensure that we will not change the CFG of this Function.
  // The only thing we need to modify is inserting boundary instr as
  // appropriate.

  computeReversePostOrder(MF, *dt, sequence);
  bool changed = false;

  unsigned round = 1;

  do {
    antiDeps.clear();

    // Step#2: visits register operand of each machine instr in the program sequence.
    for (auto &mbb : sequence) {
      auto mi = mbb->instr_begin();
      auto mie = mbb->instr_end();
      for (; mi != mie; ++mi) {
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

    DEBUG(llvm::errs()<<"\n**************** Round #"<<round<<" ***************\n"; dump(););

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

    li->runOnMachineFunction(MF);
    regions->releaseMemory();
    regions->runOnMachineFunction(MF);
    for (auto &mbb : sequence) {
      auto mi = mbb->instr_begin();
      auto mie = mbb->instr_end();
      for (; mi != mie; ++mi) {
        // Step#3: collects reg definition information.
        // Step#4: collects reg uses information.
        SmallVectorImpl<IdempotentRegion *> Regions(10);
        regions->getRegionsContaining(*mi, &Regions);
        collectRefDefUseInfo(mi, &Regions);
      }
    }

    DEBUG(llvm::errs()<<"\n      ****** Round #"<<round<<" Result *****\n"; dump(););
    ++round;

    // FIXME, cleanup is needed for transforming some incorrect code into normal status.
    bool localChanged;
    do {
      localChanged = scavengerIdem();
      changed |= localChanged;
    }while(localChanged);

  }while (!antiDeps.empty() && round < 4);

  DEBUG(llvm::errs()<<"\n************* After register renaming *************:\n";
  MF.dump(););

  li->runOnMachineFunction(MF);
  regions->releaseMemory();
  regions->runOnMachineFunction(MF);
  for (auto &mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto mie = mbb->instr_end();
    for (; mi != mie; ++mi) {
      // Step#3: collects reg definition information.
      // Step#4: collects reg uses information.
      SmallVectorImpl<IdempotentRegion *> Regions(10);
      regions->getRegionsContaining(*mi, &Regions);
      collectRefDefUseInfo(mi, &Regions);
    }
  }

  // FIXME, we should check whether anti-dependence will occurs after inserting a move instr.
  assert(antiDeps.empty() && "Anti-dependences exist!");

  DEBUG(llvm::errs()<<"\n************* After Scavenger *************:\n";
  MF.dump(););
  return changed;
}