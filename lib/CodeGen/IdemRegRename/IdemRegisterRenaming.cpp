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

using namespace llvm;

/// @author Jianping Zeng.
namespace {

  struct AntiDepPair {
    MachineOperand *use;
    MachineOperand *def;
    AntiDepPair(MachineOperand *_use, MachineOperand *_def) : use(_use), def(_def) {}

    bool operator == (AntiDepPair rhs) {
      return use == rhs.use && def == rhs.def;
    }

    bool operator != (AntiDepPair rhs) {
      return !(*this == rhs);
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
      delete scavenger;
    }
  private:
    void collectRefDefUseInfo(MachineInstr *mi, SmallVectorImpl<IdempotentRegion *> *Regions);
    bool shouldRename(AntiDepPair pair);
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
    void insertMoveAndBoundary(AntiDepPair &pair);

    void insertBoundaryAsNeed(MachineInstr *&useMI,
                              unsigned defReg);

    bool handleMultiDepsWithinSameMI(AntiDepPair &pair);

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
                       std::set<MachineOperand *> *uses,
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


static bool predEq(const MachineOperand *o1, const MachineOperand *o2)
{ return o1->getReg() == o2->getReg(); }

void RegisterRenaming::collectRefDefUseInfo(MachineInstr *mi,
                                            SmallVectorImpl<IdempotentRegion *> *Regions) {
  if (!mi) return;
  std::set<MachineOperand*> uses;
  std::set<MachineOperand*> defs;
  getDefUses(mi, &defs, &uses, allocaSet);

  if (regions->isRegionEntry(*mi)) {
    prevDefRegs[mi] = std::set<MachineOperand*>();
    prevUseRegs[mi] = uses;
  }
  else {
    // if the mi is the first mi of basic block with preds.
    if (mi == &mi->getParent()->front()) {
      MachineBasicBlock *mbb = mi->getParent();
      if (mbb->pred_empty()) {
        prevDefRegs[mi] = std::set<MachineOperand*>();
        prevUseRegs[mi] = uses;
      }
      else {
        std::set<MachineOperand*> &predDefs = prevDefRegs[mi];
        std::set<MachineOperand*> &predUses = prevUseRegs[mi];

        auto itr = mbb->pred_begin();
        auto end = mbb->pred_end();
        for (; itr != end; ++itr) {
          MachineBasicBlock *pred = *itr;
          MachineInstr *predMI = &pred->back();
          if (!regionContains(Regions, predMI))
            continue;

          std::set<MachineOperand*> localUses;
          std::set<MachineOperand*> localDefs;
          predMI->dump();
          getDefUses(predMI, &localDefs, &localUses, allocaSet);

          predDefs = Union(localDefs, prevDefRegs[predMI], predEq);
          predUses = Union(localUses, prevUseRegs[predMI]);
        }

        predUses.insert(uses.begin(), uses.end());
      }
    }
    else {
      // otherwise
      std::set<MachineOperand*> localPrevDefs;
      MachineInstr *prevMI = getPrevMI(mi);
      assert(prevMI && "previous machine instr can't be null!");
      getDefUses(prevMI, &localPrevDefs, 0, allocaSet);
      prevDefRegs[mi] = Union(prevDefRegs[prevMI], localPrevDefs, predEq);
      prevUseRegs[mi] = Union(prevUseRegs[prevMI], uses);
    }
  }

  //
  // unique the defs register operand by checking if it have same register.
  // (void)std::unique(defs->begin(), defs->end(),
  //                  [](MachineOperand* def1, MachineOperand* def2)
  //                  { return def1->getReg() == def2->getReg(); });

  if (defs.empty())
    return;
  std::set<MachineOperand *> &prevUses = prevUseRegs[mi];
  unsigned i = 0;
  for (auto defMO : defs) {
    for (MachineOperand *mo : prevUses) {
      // we don't care those anti-dependence whose def and use are not  belong to
      // the same idempotence region.
      if (!regionContains(Regions, mo->getParent()))
        continue;
      if (mo->isReg() && mo->getReg() == defMO->getReg() &&
          !contain(prevDefRegs[mo->getParent()], mo, predEq)) {

        llvm::errs()<<"["<<li->mi2Idx[mi]<<", "<<
                    li->mi2Idx[mo->getParent()]<<
                    ", "<< tri->getName(defMO->getReg())<<"]\n";
        antiDeps.emplace_back(mo, defMO);
      }
    }
    ++i;
  }
}

bool RegisterRenaming::shouldRename(AntiDepPair pair) {
  auto use = pair.use;
  MachineInstr *useMI = use->getParent();
  std::set<MachineOperand*> &defs = prevDefRegs[useMI];
  return !contain(defs, use, predEq);
}

void RegisterRenaming::spillOutInterval(LiveIntervalIdem *interval) {
  /// FIXME, I don't  know whether spilling code in here will arise no idempotence
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
          mo->getReg(), true, frameIndex, rc, tri);
      getNextMI(mi)->getOperand(0).setIsUndef(true);

      // Inserts a boundary instruction immediately after the store to partition the
      // region into two different parts for avoiding violating idempotence.
      tii->emitIdemBoundary(*mi->getParent(), getNextMI(st));

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
  interval.dump(*const_cast<TargetRegisterInfo*>(tri));
  llvm::errs()<<"\n";);

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

        //FIXME, we should check whether anti-dependence will occurs after inserting a move instr.
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

unsigned RegisterRenaming::choosePhysRegForRenaming(MachineOperand *use,
                                                    LiveIntervalIdem *interval) {
  auto rc = tri->getMinimalPhysRegClass(use->getReg());
  auto allocSet = tri->getAllocatableSet(*mf);

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

  DEBUG(for (int i = allocSet.find_first(); i > 0; i = allocSet.find_next(i)) {
    llvm::errs()<<tri->getName(i)<<" ";
  }
  llvm::errs()<<"\n";);

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
}

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

void RegisterRenaming::insertMoveAndBoundary(AntiDepPair &pair) {
  auto useMI = pair.use->getParent();

  LiveIntervalIdem *interval = new LiveIntervalIdem;
  auto to = li->getIndex(useMI);
  auto from = to - 2;

  interval->addRange(from, to);    // add an interval for a temporal move instr.
  unsigned phyReg = choosePhysRegForRenaming(pair.use, interval);

  assert(TargetRegisterInfo::isPhysicalRegister(phyReg));
  assert(phyReg != pair.use->getReg());

  // Step#8: substitute the old reg with phyReg,
  // and remove other anti-dep on this use.
  unsigned oldReg = pair.use->getReg();
  clearAntiDeps(pair.use);

  pair.use->setReg(phyReg);

  MachineBasicBlock *parent = useMI->getParent();

  // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
  //insertBoundaryAsNeed(useMI, phyReg);
  MachineBasicBlock *mbb = useMI->getParent();
  // Inserts two boundary instruction to surround the move instr.
  tii->emitIdemBoundary(*mbb, useMI);
  tii->emitIdemBoundary(*mbb, useMI);
  useMI = getPrevMI(useMI);

  assert(tii->isIdemBoundary(useMI) && "the mi at inserted position must be a splitting boundary!");
  // Step#10: insert a move instruction before splitting boundary instr.
  // This instruction would be the last killer of src reg in this copy instr.
  tii->copyPhysReg(*parent, useMI, DebugLoc(), phyReg, oldReg, true);

  // annotate the undef flag to the src reg if src reg is liveIn.
  auto copyMI = getPrevMI(useMI);
  copyMI->getOperand(1).setIsUndef(true);
}

bool RegisterRenaming::handleMultiDepsWithinSameMI(AntiDepPair &pair) {
  if (pair.use->getParent() != pair.def->getParent())
    return false;

  auto mi = pair.use->getParent();
  std::set<MachineOperand*> defs;
  std::set<MachineOperand*> uses;
  getDefUses(mi, &defs, &uses, allocaSet);

  // Collects all anti-dependences within the same mi.
  std::vector<AntiDepPair> list;
  for (auto &defMO : defs) {
    for (auto &useMO : uses) {
      if (predEq(defMO, useMO))
        list.emplace_back(AntiDepPair(useMO, defMO));
    }
  }
  // If there is not multiple anti-dependences within the same mi, use other function
  // to handle this case.
  if (list.size() <= 1)
    return false;

  auto itr = antiDeps.begin();
  auto end = antiDeps.end();
  for (; itr != end; ++itr) {
    if (std::count(list.begin(), list.end(), *itr) > 0) {
      antiDeps.erase(itr);
      --itr;
    }
  }

  // Inserts some move instr right before use MI and surrounds it with splitting boundary.
  assert(list.size() <= NUM - 2 && "Can't handle too many anti-dependences within the same MI!");
  MachineBasicBlock *parent = mi->getParent();

  // An optimization tricky: if there is a splitting boundary exists, no insert splitting.
  //insertBoundaryAsNeed(useMI, phyReg);
  MachineBasicBlock *mbb = mi->getParent();
  // Inserts two boundary instruction to surround the move instr.
  tii->emitIdemBoundary(*mbb, mi);
  tii->emitIdemBoundary(*mbb, mi);
  auto boundary = getPrevMI(mi);

  // the end index of last copy instr to be inserted.
  unsigned to = li->mi2Idx[mi];
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

    assert(tii->isIdemBoundary(boundary) && "the mi at inserted position must be a splitting boundary!");
    // Step#10: insert a move instruction before splitting boundary instr.
    // This instruction would be the last killer of src reg in this copy instr.
    tii->copyPhysReg(*parent, boundary, DebugLoc(), phyReg, oldReg, true);

    // annotate the undef flag to the src reg if src reg is liveIn.
    auto copyMI = getPrevMI(boundary);
    copyMI->getOperand(1).setIsUndef(true);
  }

  return true;
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

  // FIXME, Jianping Zeng commented on 9/12/2028.
  // If we are not going to clear the antiDeps, there is an item
  // remained produced by previous running of this pass.
  // I don't know why???
  antiDeps.clear();

  auto dt = getAnalysisIfAvailable<MachineDominatorTree>();
  assert(dt);

  std::vector<MachineBasicBlock *> sequence;
  computeReversePostOrder(MF, *dt, sequence);
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

  bool changed = false;

  DEBUG(
  li->dump(sequence);
  for (auto mbb : sequence) {
    auto mi = mbb->instr_begin();
    auto end = mbb->instr_end();
    for (; mi != end; ++mi) {
      llvm::errs()<<li->mi2Idx[mi]<<"\n uses: ";
      auto set = prevUseRegs[mi];
      for (auto mo : set) {
        llvm::errs()<<tri->getName(mo->getReg())<<", ";
      }
      llvm::errs()<<"\n";

      auto defs = prevDefRegs[mi];
      if (!defs.empty()) llvm::errs()<<"defs: ";
      for (auto defMO : defs) {
        llvm::errs()<<tri->getName(defMO->getReg())<<", ";
      }
      llvm::errs()<<"\n";
    }
  }

  for (auto pair : antiDeps) {
    llvm::errs()<<"use:"<<li->mi2Idx[pair.use->getParent()];
    pair.use->getParent()->dump();
    llvm::errs()<<"def:"<<li->mi2Idx[pair.def->getParent()];
    pair.def->getParent()->dump();
    llvm::errs()<<"\n";
  });

  // Step#5: checks if we should rename the defined register according to usd-def,
  //         If it is, construct a pair of use-def reg pair.
  simplifyAntiDeps();

  li->dump(sequence);

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
  // TODO, removes some redundant idem instruction to reduce code size.
  // FIXME, cleanup is needed for transforming some incorrect code into normal status.
  if (!scavenger)
    scavenger = new IdemInstrScavenger();
  changed |= scavenger->runOnMachineFunction(MF);
  return changed;
}