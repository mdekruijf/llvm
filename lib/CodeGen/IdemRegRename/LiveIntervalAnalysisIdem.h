#ifndef LLVM_LIVEINTERVALANALYSISIDEM_H
#define LLVM_LIVEINTERVALANALYSISIDEM_H

#include <llvm/PassSupport.h>
#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/LiveVariables.h"
#include <algorithm>
#include <llvm/Support/Debug.h>
#include <set>
#include <map>
#include "llvm/ADT/BitVector.h"

namespace llvm {
enum {
  LOAD,
  USE,
  DEF,
  STORE,
  NUM
};

class LiveRangeIdem {
public:
  unsigned start;
  unsigned end;
  LiveRangeIdem *next;

public:
  static LiveRangeIdem EndMarker;

  LiveRangeIdem(unsigned _defIdx, unsigned _killIdx, LiveRangeIdem * _next) :
      start(_defIdx), end(_killIdx), next(_next) {}
  bool contains(unsigned idx) {
    return start <= idx && idx <= end;
  }
  void print(llvm::raw_ostream &OS) {
    OS<<"["<<start<<", "<<end<<"]";
  }
  void dump() { print(llvm::errs()); }
  int intersectsAt(LiveRangeIdem *r) {

  }
};

class UsePoint {
public:
  unsigned id;
  MachineOperand *mo;
  UsePoint(unsigned ID, MachineOperand *MO) : id(ID), mo(MO) {}
  bool operator< (const UsePoint rhs) const;
};


class LiveIntervalIdem {
public:
  unsigned reg;
  LiveRangeIdem *first;
  LiveRangeIdem *last;
  std::set<UsePoint> usePoints;
  /**
   * Indicates the cost of spilling out this interval into memory.
   */
  unsigned costToSpill;

  LiveIntervalIdem() : reg(0), first(0), last(0), usePoints(), costToSpill(0) {}
  ~LiveIntervalIdem();

  std::set<UsePoint> &getUsePoint() { return usePoints; }

  void addRange(unsigned from, unsigned to);

  LiveRangeIdem *getLast() { return last; }
  void addUsePoint(unsigned numMI, MachineOperand *MO) {
    usePoints.insert(UsePoint(numMI, MO));
  }

  void print(llvm::raw_ostream &OS, const TargetRegisterInfo &tri);
  void dump(TargetRegisterInfo &TRI) { print(llvm::errs(), TRI); }
  bool isExpiredAt(unsigned pos) { return getLast()->end <= pos; }
  bool isLiveAt(unsigned pos);
  unsigned beginNumber() { return first->start; }
  unsigned endNumber() { return last->end; }
  unsigned intersectAt(LiveIntervalIdem *li) {
    return first->intersectsAt(li->first);
  }
  bool intersects(LiveIntervalIdem *cur) {
    assert(cur);
    if (cur->beginNumber() > endNumber())
      return false;

    return intersectAt(cur) != -1;
  }

  typedef std::set<UsePoint>::iterator iterator;
  iterator usepoint_begin() { return usePoints.begin(); }
  iterator usepoint_end() { return usePoints.end(); }

private:
  LiveRangeIdem *insertRangeBefore(unsigned from, unsigned to, LiveRangeIdem *cur);
};

class LiveIntervalAnalysisIdem : public MachineFunctionPass {
  std::vector<MachineInstr*> idx2MI;
  std::map<MachineInstr*, unsigned > mi2Idx;
  const TargetRegisterInfo* tri;
  std::map<unsigned, LiveIntervalIdem*> intervals;
  std::vector<std::set<unsigned> > liveIns;
  std::vector<std::set<unsigned> > liveOuts;
  llvm::BitVector allocatableRegs;
  const MachineFunction *mf;

private:
  void computeLocalLiveSet(std::vector<MachineBasicBlock *> &sequence,
                           std::vector<std::set<unsigned> > &liveGen,
                           std::vector<std::set<unsigned> > liveKill);

  void numberMachineInstr(std::vector<MachineBasicBlock*> &sequence);
  void computeGlobalLiveSet(std::vector<MachineBasicBlock*> &sequence,
                            std::vector<std::set<unsigned> > &liveIns,
                            std::vector<std::set<unsigned> > &liveOuts,
                            std::vector<std::set<unsigned> > &liveGen,
                            std::vector<std::set<unsigned> > &liveKill);
  void handleRegisterDef(unsigned reg, MachineOperand *mo, unsigned start);
  void buildIntervals(std::vector<MachineBasicBlock*> &sequence,
                      std::vector<std::set<unsigned> > &liveOuts);
public:
  static char ID;
  LiveIntervalAnalysisIdem() : MachineFunctionPass(ID) {
    initializeLiveIntervalAnalysisIdemPass(*PassRegistry::getPassRegistry());
  }

  const char *getPassName() const override {
    return "Live Interval computing for Register Renaming";
  }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const override{
    AU.addRequired<LiveVariables>();
    AU.addRequired<MachineDominatorTree>();
    AU.addRequired<MachineLoopInfo>();
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  unsigned long getNumIntervals() const { return intervals.size(); }

  typedef std::map<unsigned, LiveIntervalIdem*>::iterator interval_iterator;
  typedef std::map<unsigned, LiveIntervalIdem*>::const_iterator const_interval_iterator;

  interval_iterator interval_begin() { return intervals.begin(); }
  const_interval_iterator interval_begin() const { return intervals.begin(); }

  interval_iterator interval_end() { return intervals.end(); }
  const_interval_iterator interval_end() const { return intervals.end(); }

  bool runOnMachineFunction(MachineFunction &MF) override;

  MachineBasicBlock *getBlockAtId(unsigned pos) {
    unsigned index = pos / NUM;
    assert(index >= 0 && index < idx2MI.size());
    return idx2MI[index]->getParent();
  }

  unsigned getIndex(MachineInstr *mi) {
    assert(mi2Idx.count(mi));
    return mi2Idx[mi];
  }

  unsigned getIndexAtMBB(unsigned id) {
    return id / NUM;
  }
  void addNewInterval(unsigned int reg, LiveIntervalIdem *pIdem);
};
}

#endif //LLVM_LIVEINTERVALANALYSISIDEM_H
