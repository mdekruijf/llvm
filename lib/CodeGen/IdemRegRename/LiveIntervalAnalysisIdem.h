#ifndef LLVM_LIVEINTERVALANALYSISIDEM_H
#define LLVM_LIVEINTERVALANALYSISIDEM_H

#include <llvm/PassSupport.h>
#include <llvm/CodeGen/MachineIdempotentRegions.h>
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
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

// forward declaration.
class RangeIterator;

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

  /**
   * Determines the iterator position where this will interferes with
   * another {@link LiveRangeItem} {@code r} at the start of Live Range.
   *
   * Return the interfered iterator position which will equal to end()
   * if no interference exist.
   * @param r2
   * @return
   */
  RangeIterator intersectsAt(LiveRangeIdem *r2);
};

class UsePoint {
public:
  unsigned id;
  MachineOperand *mo;
  UsePoint(unsigned ID, MachineOperand *MO) : id(ID), mo(MO) {}
  bool operator< (const UsePoint rhs) const;
};

class RangeIterator : public std::iterator<std::forward_iterator_tag, LiveRangeIdem*> {
private:
  LiveRangeIdem *cur;
public:
  RangeIterator(LiveRangeIdem *_first) : cur(_first) {}
  RangeIterator() = default;

  RangeIterator &operator++() {
    cur = cur->next;
    return *this;
  }
  RangeIterator operator++(int) {
    RangeIterator res = *this;
    ++res;
    return res;
  }
  bool operator ==(RangeIterator itr) {
    return cur == itr.cur;
  }
  bool operator !=(RangeIterator itr) { return !(*this == itr); }
  LiveRangeIdem *operator->() {
    assert(cur != &llvm::LiveRangeIdem::EndMarker);
    return cur;
  }
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

  LiveIntervalIdem() : reg(0), first(&LiveRangeIdem::EndMarker),
                       last(&LiveRangeIdem::EndMarker),
                       usePoints(), costToSpill(0) {}
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

  RangeIterator intersectAt(LiveIntervalIdem *li);
  bool intersects(LiveIntervalIdem *cur);

  typedef std::set<UsePoint>::iterator iterator;
  iterator usepoint_begin() { return usePoints.begin(); }
  iterator usepoint_end() { return usePoints.end(); }

private:
  /**
   * Insert live range before the current range. It will merge range to be inserted with
   * adjacent range as necessary.
   * @param from
   * @param to
   * @param cur
   */
  void insertRangeBefore(unsigned from, unsigned to, LiveRangeIdem *&cur);

public:
  friend class RangeIterator;

  RangeIterator begin() { return {first}; }
  RangeIterator end() { return {last}; }
  const RangeIterator begin() const { return RangeIterator(first); }
  const RangeIterator end() const { return RangeIterator(last); }
};

class LiveIntervalAnalysisIdem : public MachineFunctionPass {
public:
  /**
   * Map from instruction slot to corresponding machine instruction.
   */
  std::vector<MachineInstr*> idx2MI;
  /**
   * Map from machine instruction to it's number.
   */
  std::map<MachineInstr*, unsigned > mi2Idx;
  const TargetRegisterInfo* tri;
  /**
   * Map from original physical register to it's corresponding
   * live interval.
   */
  std::map<unsigned, LiveIntervalIdem*> intervals;
  std::vector<std::set<unsigned> > liveIns;
  std::vector<std::set<unsigned> > liveOuts;
  llvm::BitVector allocatableRegs;
  const MachineFunction *mf;
  MachineDominatorTree *dt;
  MachineLoopInfo *loopInfo;

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
  /**
   * Weight each live interval by computing the use number of live interval
   * according to it's loop nesting depth.
   */
  void weightLiveInterval();

public:
  static char ID;
  LiveIntervalAnalysisIdem() : MachineFunctionPass(ID) {
    initializeLiveIntervalAnalysisIdemPass(*PassRegistry::getPassRegistry());
  }

  const char *getPassName() const override {
    return "Live Interval computing for Register Renaming";
  }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const override{
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
    assert(index < idx2MI.size());
    return idx2MI[index]->getParent();
  }

  unsigned getIndex(MachineInstr *mi) {
    assert(mi2Idx.count(mi));
    return mi2Idx[mi];
  }

  unsigned getIndexAtMBB(unsigned id) {
    return id / NUM;
  }
  void insertOrCreateInterval(unsigned int reg, LiveIntervalIdem *pIdem);

  void dump(std::vector<MachineBasicBlock *> &sequence);
};
}

#endif //LLVM_LIVEINTERVALANALYSISIDEM_H
