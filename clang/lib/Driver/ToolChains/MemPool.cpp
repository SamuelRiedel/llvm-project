//===--- MemPool.cpp - MemPool ToolChain Implementations --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MemPool.h"
#include "CommonArgs.h"
#include "InputInfo.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/Options.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/Process.h"
#include "llvm/Support/raw_ostream.h"

using namespace clang::driver;
using namespace clang::driver::toolchains;
using namespace clang::driver::tools;
using namespace clang;
using namespace llvm::opt;

/// MemPool Toolchain
// methods are adapted from RISCV.cpp
MemPoolToolChain::MemPoolToolChain(const Driver &D, const llvm::Triple &Triple,
                                   const ArgList &Args)
    : Generic_ELF(D, Triple, Args) {
  GCCInstallation.init(Triple, Args);
  getFilePaths().push_back(computeSysRoot() + "/lib");
  if (GCCInstallation.isValid()) {
    getFilePaths().push_back(GCCInstallation.getInstallPath().str());
    getProgramPaths().push_back(
        (GCCInstallation.getParentLibPath() + "/../bin").str());
  }
}

Tool *MemPoolToolChain::buildLinker() const {
  return new tools::MemPool::Linker(*this);
}

void MemPoolToolChain::addClangTargetOptions(
    const llvm::opt::ArgList &DriverArgs, llvm::opt::ArgStringList &CC1Args,
    Action::OffloadKind) const {
  CC1Args.push_back("-nostdsysteminc");
  CC1Args.push_back("-fuse-init-array");
  CC1Args.push_back("-mllvm");
  CC1Args.push_back("-enable-misched");
}

void MemPoolToolChain::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                                 ArgStringList &CC1Args) const {
  if (DriverArgs.hasArg(options::OPT_nostdinc))
    return;

  if (!DriverArgs.hasArg(options::OPT_nostdlibinc)) {
    SmallString<128> Dir(computeSysRoot());
    llvm::sys::path::append(Dir, "include");
    addSystemInclude(DriverArgs, CC1Args, Dir.str());
  }
}

// void MemPoolToolChain::addLibStdCxxIncludePaths(
//     const llvm::opt::ArgList &DriverArgs,
//     llvm::opt::ArgStringList &CC1Args) const {
//   const GCCVersion &Version = GCCInstallation.getVersion();
//   StringRef TripleStr = GCCInstallation.getTriple().str();
//   const Multilib &Multilib = GCCInstallation.getMultilib();
//   addLibStdCXXIncludePaths(computeSysRoot() + "/include/c++/" + Version.Text,
//                            "", TripleStr, "", "", Multilib.includeSuffix(),
//                            DriverArgs, CC1Args);
// }

std::string MemPoolToolChain::computeSysRoot() const {
  if (!getDriver().SysRoot.empty())
    return getDriver().SysRoot;

  if (!GCCInstallation.isValid())
    return std::string();

  StringRef LibDir = GCCInstallation.getParentLibPath();
  StringRef TripleStr = GCCInstallation.getTriple().str();
  std::string SysRootDir = LibDir.str() + "/../" + TripleStr.str();

  if (!llvm::sys::fs::exists(SysRootDir))
    return std::string();

  return SysRootDir;
}

// MemPool::Linker::Linker(const ToolChain &TC) : GnuTool("MemPool::Linker",
// "ld", TC) {
//   llvm::Optional<std::string> MemPoolSdkDir =
//       llvm::sys::Process::GetEnv("MEMPOOL_SDK_DIR");
//   if (MemPoolSdkDir.hasValue()) {
//     this->MemPoolSdkDir = MemPoolSdkDir.getValue();
//   }
// }

llvm::opt::DerivedArgList *
MemPoolToolChain::TranslateArgs(const llvm::opt::DerivedArgList &Args,
                                StringRef BoundArch,
                                Action::OffloadKind DeviceOffloadKind) const {
  return nullptr;
}

void MemPool::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                                   const InputInfo &Output,
                                   const InputInfoList &Inputs,
                                   const ArgList &Args,
                                   const char *LinkingOutput) const {
  const ToolChain &ToolChain = getToolChain();
  const Driver &D = ToolChain.getDriver();
  ArgStringList CmdArgs;

  if (!D.SysRoot.empty())
    CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

  std::string Linker = getToolChain().GetProgramPath(getShortName());

  bool WantCRTs =
      !Args.hasArg(options::OPT_nostdlib, options::OPT_nostartfiles);

  if (WantCRTs) {
    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crt0.o")));
    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crtbegin.o")));
  }

  Args.AddAllArgs(CmdArgs, options::OPT_L);
  ToolChain.AddFilePathLibArgs(Args, CmdArgs);
  Args.AddAllArgs(CmdArgs,
                  {options::OPT_T_Group, options::OPT_e, options::OPT_s,
                   options::OPT_t, options::OPT_Z_Flag, options::OPT_r});

  AddLinkerInputs(ToolChain, Inputs, Args, CmdArgs, JA);

  // TODO: add C++ includes and libs if compiling C++.

  if (!Args.hasArg(options::OPT_nostdlib) &&
      !Args.hasArg(options::OPT_nodefaultlibs)) {
    if (ToolChain.ShouldLinkCXXStdlib(Args))
      ToolChain.AddCXXStdlibLibArgs(Args, CmdArgs);
    CmdArgs.push_back("--start-group");
    CmdArgs.push_back("-lc");
    CmdArgs.push_back("-lgloss");
    CmdArgs.push_back("--end-group");
    CmdArgs.push_back("-lgcc");
  }

  if (WantCRTs)
    CmdArgs.push_back(Args.MakeArgString(ToolChain.GetFilePath("crtend.o")));

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());
  C.addCommand(llvm::make_unique<Command>(JA, *this, Args.MakeArgString(Linker),
                                          CmdArgs, Inputs));
}

// void MemPool::Linker::ConstructJob(Compilation &C, const JobAction &JA,
//                                 const InputInfo &Output,
//                                 const InputInfoList &Inputs,
//                                 const ArgList &Args,
//                                 const char *LinkingOutput) const {
//   const ToolChain &ToolChain = getToolChain();
//   const Driver &D = ToolChain.getDriver();
//   ArgStringList CmdArgs;
//   // CmdArgs.push_back("-march=elf32lriscv");

//   if (!D.SysRoot.empty())
//     CmdArgs.push_back(Args.MakeArgString("--sysroot=" + D.SysRoot));

//   std::string Linker = getToolChain().GetProgramPath(getShortName());

//   SmallString<128> RtConf(this->MemPoolSdkDir);
//   llvm::sys::path::append(
//       RtConf, "pulp-sdk/pkg/sdk/dev/install/lib/mempool-z-7045/rt/crt0.o");
//   CmdArgs.push_back(Args.MakeArgString(RtConf));

//   SmallString<128> Crt0(this->MemPoolSdkDir);
//   llvm::sys::path::append(
//       Crt0, "pulp-sdk/pkg/sdk/dev/install/mempool/mempool-z-7045/rt_conf.o");
//   CmdArgs.push_back(Args.MakeArgString(Crt0));

//   CmdArgs.push_back("-nostdlib");
//   CmdArgs.push_back("-nostartfiles");
//   CmdArgs.push_back("--gc-sections");

//   Args.AddAllArgs(CmdArgs, options::OPT_L);
//   ToolChain.AddFilePathLibArgs(Args, CmdArgs);
//   Args.AddAllArgs(CmdArgs,
//                   {options::OPT_T_Group, options::OPT_e, options::OPT_s,
//                    options::OPT_t, options::OPT_Z_Flag, options::OPT_r});

//   AddLinkerInputs(ToolChain, Inputs, Args, CmdArgs, JA);

//   SmallString<128> ArgStr("-T");
//   ArgStr.append(this->MemPoolSdkDir);
//   llvm::sys::path::append(
//       ArgStr,
//       "pulp-sdk/pkg/sdk/dev/install/mempool/mempool-z-7045/omptarget.ld");
//   CmdArgs.push_back(Args.MakeArgString(ArgStr));

//   ArgStr.clear();
//   ArgStr.append("-T");
//   ArgStr.append(this->MemPoolSdkDir);
//   llvm::sys::path::append(
//       ArgStr,
//       "pulp-sdk/pkg/sdk/dev/install/mempool/mempool-z-7045/test_config.ld");
//   CmdArgs.push_back(Args.MakeArgString(ArgStr));

//   ArgStr.clear();
//   ArgStr.append("-L");
//   ArgStr.append(this->MemPoolSdkDir);
//   llvm::sys::path::append(ArgStr,
//                           "pulp-sdk/pkg/sdk/dev/install/lib/mempool-z-7045");
//   CmdArgs.push_back(Args.MakeArgString(ArgStr));

//   ArgStr.clear();
//   ArgStr.append("-L");
//   ArgStr.append(this->MemPoolSdkDir);
//   llvm::sys::path::append(
//       ArgStr,
//       "mempool-gcc-toolchain/install/lib/gcc/riscv32-unknown-elf/7.1.1");
//   CmdArgs.push_back(Args.MakeArgString(ArgStr));

//   ArgStr.clear();
//   ArgStr.append("-L");
//   ArgStr.append(this->MemPoolSdkDir);
//   llvm::sys::path::append(ArgStr,
//                           "mempool-gcc-toolchain/install/riscv32-unknown-elf/lib");
//   CmdArgs.push_back(Args.MakeArgString(ArgStr));

//   // Currently no support for C++, otherwise add C++ includes and libs if
//   // compiling C++.

//   CmdArgs.push_back("-lomptarget-pulp");
//   CmdArgs.push_back("-lrt");
//   CmdArgs.push_back("-lrtio");
//   CmdArgs.push_back("-lrt");
//   CmdArgs.push_back("-lmempool-target");
//   CmdArgs.push_back("-lvmm");
//   CmdArgs.push_back("-larchi_host");
//   CmdArgs.push_back("-lrt");
//   CmdArgs.push_back("-larchi_host");
//   CmdArgs.push_back("-lgcc");
//   CmdArgs.push_back("-lbench");
//   CmdArgs.push_back("-lm");
//   CmdArgs.push_back("-lpremnotify");

//   CmdArgs.push_back("-o");
//   CmdArgs.push_back(Output.getFilename());

//   C.addCommand(llvm::make_unique<Command>(JA, *this,
//   Args.MakeArgString(Linker),
//                                           CmdArgs, Inputs));
// }
