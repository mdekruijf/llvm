//===-------- IdempotenceOptions.cpp ----------------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains idempotence-specific compilation options.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/IdempotenceOptions.h"
#include "llvm/Support/CommandLine.h"
using namespace llvm;

namespace llvm {

cl::opt<bool> IdempotenceVerify(
    "idempotence-verify", cl::Hidden,
    cl::desc("Verify region construction and idempotence preservation"),
    cl::init(false));

cl::opt<IdempotenceOptions::ConstructionMode> IdempotenceConstructionMode(
    "idempotence-construction", cl::Hidden,
    cl::desc("Idempotent region construction mode"),
    cl::values(clEnumValN(IdempotenceOptions::NoConstruction, 
                          "none", "No region construction"),
               clEnumValN(IdempotenceOptions::OptimizeForSize, 
                          "size", "Construct optimized for size"),
               clEnumValN(IdempotenceOptions::OptimizeForSpeed,
                          "speed", "Construct optimized for speed"),
               clEnumValN(IdempotenceOptions::BranchRecovery,
                          "branch", "Construct for branch recovery"),
               clEnumValEnd),
    cl::init(IdempotenceOptions::NoConstruction));

cl::opt<IdempotenceOptions::PreservationMode> IdempotencePreservationMode(
    "idempotence-preservation", cl::Hidden,
    cl::desc("Idempotence preservation mode"),
    cl::values(clEnumValN(IdempotenceOptions::NoPreservation,
                          "none", "Do not preserve idempotence"),
               clEnumValN(IdempotenceOptions::VariableCF,
                          "vcf", "Preserve assuming variable control flow"),
               clEnumValN(IdempotenceOptions::InvariableCF,
                          "icf", "Preserve assuming invariable control flow"),
               clEnumValEnd),
    cl::init(IdempotenceOptions::NoPreservation));

cl::opt<bool> EnableRegisterRenaming(
    "enable-reg-renaming-idem", cl::Hidden,
    cl::desc("Indicates if enable register renaming or not"),
    cl::init(false));

cl::opt<bool> RenamingIdemVerify(
    "renaming-idem-verify", cl::Hidden,
    cl::desc("Verify idempotence preservation after register renaming"),
    cl::init(false));

cl::opt<bool> EliminateIdemBoundary(
    "eliminate-idem-boundary", cl::Hidden,
    cl::desc("Eliminate all idem boundaries for performance evaluation"),
    cl::init(false));

cl::opt<bool> EnableIdemStatistic("enable-idem-statistic", cl::Hidden,
    cl::desc("Report the statistic data for idempotence regions, such as number of regions, average length"),
    cl::init(false));

cl::opt<std::string> IdemStatisticOutFile("idem-stat-outfile",
    cl::Hidden,
    cl::value_desc("idem-stat-outfile"),
    cl::desc("path to output file for idem statistic, default by /tmp/idem_stat1x32/idemStat.txt"),
    cl::init("/tmp/idem_stat1x32/idemStat.txt"));

std::string moduleName;

} // namespace llvm

