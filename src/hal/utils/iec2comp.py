#!/usr/bin/env python3
#    iec2comp: an IEC 61131-3 program (Structured Text / SFC) as a HAL component.
#    Copyright (C) 2026 SyncTwin <https://github.com/SyncTwin>, yurc
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 2 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
"""iec2comp: program.st -> MATIEC iec2c C -> one halcompile .comp that runs the program in a HAL thread.

The program is compiled by the MATIEC compiler (iec2c, an external program: on PATH or in $MATIEC with its
lib/ next to it, as the MATIEC build tree and Beremiz lay it out).  The generated C goes after ';;' of one
.comp: Res.c with POUS.h/Cfg.h/POUS.c pasted in place of their #include (one source file per component);
Cfg.c is not used, the component calls RES_init__/RES_run__ itself, one RESOURCE, one TASK per thread period.
The MATIEC runtime headers (lib/C/*.h) are written next to the .comp; build with
'halcompile --install <dir>/iec_<prog>.comp' (or --compile): halcompile adds the directory of the .comp
to the include path.

Pins of the component (instance <comp>.0):
  permit                 0 = the program is stopped and the outputs are at their -safe values
  enable                 1 = run the program (run = enable AND permit; a hook may supply enable instead)
  ix-a-b / qx-a-b        located BOOL %IXa.b / %QXa.b; qx-a-b-safe is the output while stopped
  ib|iw|id-a-b, qb|qw|qd-a-b
                         located words: s32 for signed IEC types, u32 for unsigned and bit strings, float
                         for REAL (64 bit refused: HAL has no pin that wide); %M* is program memory, no pin
  ix-a-b-force/-fval/-armed, forced
                         force table: a rising edge of -force while permit = 1 arms it, the program then
                         reads -fval; permit = 0 drops every force
  step-<s>               SFC step <s>.X
  knob-<name>            VAR_INPUT of the program (REAL/LREAL/INT/DINT/BOOL), default from the program text
  axN-*                  PLCopen MC_* function blocks (lib/iec/mc_plcopen.st) on axis N: wired to mc-axis.N
                         (mc_axis(9)); AXIS_REF variables of the program get AxisNo from the axes map.
__CURRENT_TIME (timers, step.T) is the sum of the thread period while the program runs.
uspace loads components RTLD_GLOBAL, so every MATIEC symbol is under '#pragma GCC visibility push(hidden)':
two programs in one HAL do not see each other's RES__INST0, iec_mc, __CURRENT_TIME.

Use as a module: compile_program() then render(); Hooks lets a caller add ST before the program, its own
pins, C before/after the scan and run/hold/drop conditions without changing this file.
"""
from __future__ import annotations

import argparse
import functools
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

#: iec2c may not exit after some syntax errors; the errors are already printed by then
IEC2C_TIMEOUT_S = 30
DEFAULT_THREAD = "servo-thread"
_KNOB_TYPES = {"REAL": "float", "LREAL": "float", "INT": "s32", "DINT": "s32", "BOOL": "bit"}
#: located word -> HAL pin: signedness from the IEC type of the variable, not from the size letter
_WORD_HAL = {"SINT": "s32", "INT": "s32", "DINT": "s32", "USINT": "u32", "UINT": "u32", "UDINT": "u32",
             "BYTE": "u32", "WORD": "u32", "DWORD": "u32", "REAL": "float"}
_LOCATED = re.compile(r"__INIT_LOCATED\((\w+),__([IQM])([XBWDL])(\d+)_(\d+),data__->(\w+),")
_PRAGMA = re.compile(r"\{attribute[^}]*\}")
_ST_COMMENT = re.compile(r"\(\*.*?\*\)|//[^\n]*", re.S)
_IEC2C_AT = re.compile(r"^(.*?\.st:(\d+)-(\d+)\.\.[\d-]+:\s*error:.*)$", re.M)
_LIB_INCLUDE = re.compile(r'\{#include\s+"([^"]+)"\s*\}')
MC_LIBRARY = "mc_plcopen.st"


class IecCompError(Exception):
    """The program does not become a component; the text says why (iec2c errors as 'file:line-col..: error')."""


def _here() -> Path:
    return Path(os.path.realpath(__file__)).parent


def library_dir() -> Path:
    """Directory of mc_plcopen.st: $IEC2COMP_LIB, the source tree (src/hal/utils -> lib/iec), a run-in-place
    bin/ (-> lib/iec) or an installed bin/ (-> share/linuxcnc/iec)."""
    cands = [os.environ.get("IEC2COMP_LIB", ""), _here() / "../../../lib/iec", _here() / "../lib/iec",
             _here() / "../share/linuxcnc/iec"]
    for c in cands:
        if c and Path(c, MC_LIBRARY).is_file():
            return Path(c).resolve()
    raise IecCompError(f"{MC_LIBRARY} not found (set IEC2COMP_LIB)")


def find_matiec() -> tuple[str, str]:
    """iec2c and its lib/ (ieclib.txt, C/*.h): $MATIEC (the MATIEC build directory) or iec2c on PATH."""
    exe = os.path.join(os.environ["MATIEC"], "iec2c") if os.environ.get("MATIEC") else shutil.which("iec2c")
    if not exe or not os.path.isfile(exe):
        raise IecCompError("iec2c (MATIEC) not found: put it on PATH or set MATIEC to its build directory")
    lib = os.path.join(os.path.dirname(os.path.realpath(exe)), "lib")
    if not os.path.isfile(os.path.join(lib, "ieclib.txt")):
        raise IecCompError(f"no MATIEC library next to iec2c ({lib}/ieclib.txt)")
    return exe, lib


def source(st: str, axes: dict[str, int], period_ns: int, *, prelude: str = "",
           programs: list[tuple[str, str, str]] = (), globals_: list[str] = ()) -> str:
    """Text for iec2c: MC_* library + prelude + the program without pragmas + extra programs + CONFIGURATION
    (one RESOURCE, TASK INTERVAL = the thread period, VAR_GLOBAL x : AXIS_REF := (AxisNo := N) for each
    AXIS_REF of VAR_EXTERNAL, then globals_). programs: (instance, POU type, ST text) run in the same TASK
    right after the program."""
    src = _PRAGMA.sub("", st)
    m = re.search(r"\bPROGRAM\s+(\w+)", src)
    if not m:
        raise IecCompError("no PROGRAM in the source")
    ext = re.search(r"^\s*VAR_EXTERNAL(.*?)END_VAR", src, re.S | re.M)
    declared = re.findall(r"(\w+)\s*:\s*AXIS_REF\s*;", ext.group(1)) if ext else []
    missing = [a for a in declared if a not in axes]
    if missing:
        raise IecCompError(f"axes of the program without an mc-axis number: {', '.join(missing)}")
    glob = "".join(f"    {a} : AXIS_REF := (AxisNo := {axes[a]});\n" for a in declared) + "".join(globals_)
    progs = "".join(f"  PROGRAM {i} WITH Main : {t};\n" for i, t, _ in programs)
    cfg = ("CONFIGURATION Cfg\n RESOURCE Res ON PLC\n" + (f"  VAR_GLOBAL\n{glob}  END_VAR\n" if glob else "")
           + f"  TASK Main(INTERVAL := T#{period_ns / 1e6:g}ms, PRIORITY := 0);\n"
           + f"  PROGRAM inst0 WITH Main : {m.group(1)};\n{progs} END_RESOURCE\nEND_CONFIGURATION\n")
    return (Path(library_dir(), MC_LIBRARY).read_text(encoding="utf-8") + prelude + src
            + "".join(t for _, _, t in programs) + cfg)


@functools.lru_cache(maxsize=4)
def _library_names(lib: str) -> dict[str, str]:
    """Function and FB names of the MATIEC library (from its ieclib.txt and includes): iec2c refuses them as a
    variable, instance or program name and does not say why."""
    names: dict[str, str] = {}

    def walk(f: str) -> None:
        t = _ST_COMMENT.sub("", Path(lib, f).read_text(encoding="utf-8", errors="replace"))
        for m in re.finditer(r"^\s*(FUNCTION_BLOCK|FUNCTION)\s+(\w+)", t, re.M):
            names.setdefault(m.group(2).upper(), "standard function block" if m.group(1) == "FUNCTION_BLOCK"
                             else "standard function")
        for inc in _LIB_INCLUDE.findall(t):
            walk(inc)
    walk("ieclib.txt")
    return names


def name_clash_hint(err: str, text: str, lib: str) -> str:
    """To an iec2c error line that starts at a library name, add the cause in words (iec2c itself says only
    'invalid variable(s) declaration' / 'invalid program name'). A call SEL(...) is not a declaration."""
    lines, names = text.splitlines(), _library_names(lib)

    def add(m: re.Match) -> str:
        n, c = int(m.group(2)), int(m.group(3))
        ln = lines[n - 1] if 0 < n <= len(lines) else ""
        # column: 1-based, except on the first line of the file (0-based there) — the word that starts at c-1 or c
        w = next((x for o in (c - 1, c) if 0 <= o and not (o and (ln[o - 1].isalnum() or ln[o - 1] == "_"))
                  for x in [re.match(r"\w+(?!\w|\s*\()", ln[o:])] if x), None)
        kind = names.get(w.group(0).upper()) if w else None
        return m.group(1) + (f" — the name '{w.group(0)}' is taken: {kind} {w.group(0).upper()} of the "
                             "IEC 61131-3 library (MATIEC), rename it" if kind else "")
    return _IEC2C_AT.sub(add, err)


def iec2c(text: str) -> tuple[dict[str, str], dict[str, str]]:
    """Run iec2c on text -> (POUS.c/POUS.h/Res.c/Cfg.h, runtime headers lib/C/*.h)."""
    exe, lib = find_matiec()
    with tempfile.TemporaryDirectory(prefix="iec2c-") as tmp:
        src = os.path.join(tmp, "node.st")
        Path(src).write_text(text, encoding="utf-8")
        try:
            r = subprocess.run([exe, "-f", "-I", lib, "-T", tmp, src], capture_output=True, text=True,
                               timeout=IEC2C_TIMEOUT_S)
        except subprocess.TimeoutExpired as e:
            err = e.stderr or b""
            if isinstance(err, bytes):
                err = err.decode("utf-8", errors="replace")
            raise IecCompError(f"iec2c hung ({IEC2C_TIMEOUT_S} s): {err.strip()[:600] or '(no stderr)'}") from e
        if r.returncode:
            raise IecCompError(f"iec2c: {name_clash_hint((r.stderr or r.stdout).strip(), text, lib)[:1200]}")
        gen = {f: Path(tmp, f).read_text() for f in ("POUS.c", "POUS.h", "Res.c", "Cfg.h")}
    hdr = {f: Path(lib, "C", f).read_text() for f in sorted(os.listdir(os.path.join(lib, "C"))) if f.endswith(".h")}
    return gen, hdr


def _c_literal(v: str) -> str:
    m = re.match(r"__(?:REAL|LREAL|INT|DINT)_LITERAL\((.*)\)$", v)
    if m:
        return m.group(1)
    return {"__BOOL_LITERAL(TRUE)": "1", "__BOOL_LITERAL(FALSE)": "0"}.get(v, v)


@dataclass
class Program:
    """A compiled program. located/words are lists the caller may extend before render()
    (located: (IEC type, 'I'|'Q', a, b, variable); words: (IEC type, 'I'|'Q'|'M', size letter, a, b, variable))."""
    name: str                                   # PROGRAM type, upper case
    st: str
    axes: dict[str, int]
    period_ns: int
    gen: dict[str, str]
    hdr: dict[str, str]
    located: list[tuple]
    words: list[tuple]
    steps: list[str]                            # SFC steps as iec2c spells them (upper case)
    knobs: list[tuple[str, str]]                # (VAR_INPUT name, IEC type)
    init: dict[str, str]                        # initial values iec2c printed
    set_position: bool                          # the program calls MC_SetPosition
    power: bool                                 # the program calls MC_Power
    found: list[str] = field(default_factory=list)   # POU types of `programs` present in the C


def compile_program(st: str, axes: dict[str, int], period_ns: int, *, prelude: str = "",
                    programs: list[tuple[str, str, str]] = (), globals_: list[str] = ()) -> Program:
    """program.st -> iec2c -> parsed interface. Arguments as source()."""
    gen, hdr = iec2c(source(st, axes, period_ns, prelude=prelude, programs=programs, globals_=globals_))
    pous_c, pous_h = gen["POUS.c"], gen["POUS.h"]
    plain = _PRAGMA.sub("", st)
    prog = re.search(r"\bPROGRAM\s+(\w+)", plain).group(1).upper()
    body = pous_c[pous_c.index(f"void {prog}_init__"):]
    every = _LOCATED.findall(body)
    if len(every) != body.count("__INIT_LOCATED("):
        raise IecCompError("AT % address: two fields only (%IX0.3, %QW0.1), a deeper address gets no pin")
    loc = [(t, k, a, b, v) for t, k, s, a, b, v in every if s == "X" and k != "M"]
    wloc = [(t, k, s, a, b, v) for t, k, s, a, b, v in every if s != "X" or k == "M"]
    for t, k, s, a, b, v in wloc:
        if k != "M" and (s == "L" or t not in _WORD_HAL):
            raise IecCompError(f"'{v.lower()}' AT %{k}{s}{a}.{b} : {t} — "
                               + ("64 bit: HAL has no pin that wide" if s == "L" or t in ("LINT", "ULINT", "LWORD")
                                  else f"type {t} has no HAL pin (there are: {', '.join(_WORD_HAL)})"))
    found = [t for _i, t, _s in programs if f"void {t.upper()}_init__" in pous_c]
    for t in found:   # addresses of the extra programs join the program's, one pin per address
        loc += [(ty, k, a, b, v) for ty, k, _s, a, b, v in _LOCATED.findall(pous_c[pous_c.index(f"void {t.upper()}_init__"):])]
    if found:
        first: dict = {}
        loc = [first.setdefault(x[1:4], x) for x in loc if x[1:4] not in first]   # first declaration wins
    steps = re.findall(r"__DECLARE_VAR\(BOOL,(\w+)_X\)", pous_h[pous_h.index(f"__DECLARE_PROGRAM_TYPE({prog},"):])
    prog_st = plain[re.search(r"\bPROGRAM\s+\w+", plain).start():]   # knobs of the program, not of a library FB
    var_in = re.search(r"^\s*VAR_INPUT(.*?)END_VAR", prog_st, re.S | re.M)
    knobs = [(n.upper(), t.upper()) for n, t in re.findall(r"(\w+)\s*:\s*(\w+)", var_in.group(1) if var_in else "")
             if t.upper() in _KNOB_TYPES]
    return Program(name=prog, st=st, axes=dict(axes), period_ns=period_ns, gen=gen, hdr=hdr, located=loc,
                   words=wloc, steps=steps, knobs=knobs,
                   init=dict(re.findall(r"__INIT_VAR\(data__->(\w+),(.*?),retain\)", body)),
                   set_position=bool(re.search(r"\bMC_SetPosition\b", plain, re.I)),
                   power=bool(re.search(r"\bMC_Power\b", plain, re.I)), found=found)


@dataclass
class Hooks:
    """What a caller adds around the program. C fragments are whole lines ('    stmt;\\n'); they see the pins
    (C names), `period`, `run`, iec_ax[] and the MATIEC globals (RES__INST0...)."""
    notes: list[str] = field(default_factory=lambda: ["// generated by iec2comp — do not edit"])
    pin_doc: Callable[[str, str], str] | None = None   # (pin C name, default doc) -> doc
    enable: str | None = None        # C expression enabling the program; None = pin `enable`
    pins_head: list[str] = field(default_factory=list)          # after permit/enable
    pins_after_steps: list[str] = field(default_factory=list)   # after step-*, before knob-*
    drop: str = ""                   # C expression that also drops the force table
    hold: str = ""                   # C expression that freezes the scan (outputs keep their values)
    axis_reset: str = ""             # C expression OR-ed into every axN-reset
    c_vars: str = ""                 # C variables after the located buffers (hidden)
    c_decl: str = ""                 # C declarations after the axis host (hidden)
    statics: list[str] = field(default_factory=list)            # more `static int` beside iec_ready
    c_init: str = ""                 # first scan, after RES_init__()
    c_pre: str = ""                  # every scan, before `int run`
    c_in: str = ""                   # after the inputs are copied in
    c_pre_run: str = ""              # right before the scan
    c_out: str = ""                  # after the outputs, steps
    permit_signal: str | None = None                            # net <signal> => <inst>.permit
    hal_after_load: list[str] = field(default_factory=list)     # after loadrt/addf/permit
    axis_hal: Callable[[int], list[str]] | None = None          # per axis, before the axN-state net


@dataclass
class Comp:
    comp: str
    files: dict[str, str]           # <comp>.comp + MATIEC runtime headers
    hal_lines: list[str]
    inputs: dict[str, str] = field(default_factory=dict)      # "%IX0.0" -> pin
    outputs: dict[str, str] = field(default_factory=dict)     # "%QX0.0" -> pin
    words: dict[str, str] = field(default_factory=dict)       # "%IW0.0"/"%QD0.2" -> pin
    knobs: dict[str, str] = field(default_factory=dict)       # VAR_INPUT name -> pin
    step_pins: dict[str, str] = field(default_factory=dict)   # SFC step (lower case) -> pin


def render(p: Program, comp: str, thread: str = DEFAULT_THREAD, hooks: Hooks | None = None) -> Comp:
    """Program -> the .comp text, its runtime headers and the HAL lines (loadrt, addf, mc-axis.N nets)."""
    h = hooks or Hooks()
    if not re.fullmatch(r"[a-z][a-z0-9_]*", comp):
        raise IecCompError(f"component name '{comp}': [a-z0-9_] only")
    doc = h.pin_doc or (lambda _n, d: d)
    inst = comp.replace("_", "-") + ".0"
    loc, wloc, steps, knobs, ax = p.located, p.words, p.steps, p.knobs, sorted(set(p.axes.values()))
    pins = [f'pin in bit permit "{doc("permit", "0 = program stopped, outputs at their -safe values")}";']
    if h.enable is None:
        pins.append(f'pin in bit enable "{doc("enable", "1 = run the program (AND permit)")}";')
    pins += h.pins_head
    pins += [f'pin {"in" if k == "I" else "out"} bit {k.lower()}x_{a}_{b} "%{k}X{a}.{b} {v.lower()}";'
             for _t, k, a, b, v in loc]
    pins += [f'pin {"in" if k == "I" else "out"} {_WORD_HAL[t]} {k.lower()}{s.lower()}_{a}_{b} "%{k}{s}{a}.{b} {v.lower()} : {t}";'
             for t, k, s, a, b, v in wloc if k != "M"]
    pins += [f'pin in {_WORD_HAL[t]} q{s.lower()}_{a}_{b}_safe "{doc(f"q{s.lower()}_{a}_{b}_safe", f"%Q{s}{a}.{b} while enable = 0 (0 unless set)")}";'
             for t, k, s, a, b, v in wloc if k == "Q"]
    pins += [f'pin in bit qx_{a}_{b}_safe "{doc(f"qx_{a}_{b}_safe", f"%QX{a}.{b} while enable = 0: the safe value of the output")}";'
             for _t, k, a, b, _v in loc if k == "Q"]
    # Force table: force + fval are setp'd by an operator tool, the program reads armed ? fval : ix
    pins += [x for _t, k, a, b, _v in loc if k == "I" for x in (
             f'pin in bit ix_{a}_{b}_force "{doc(f"ix_{a}_{b}_force", f"%IX{a}.{b} force: armed by a rising edge while permit = 1, dropped by permit = 0")}";',
             f'pin in bit ix_{a}_{b}_fval "{doc(f"ix_{a}_{b}_fval", f"%IX{a}.{b} forced value")}";',
             f'pin out bit ix_{a}_{b}_armed "%IX{a}.{b} force armed: the program reads fval";')]
    if any(k == "I" for _t, k, _a, _b, _v in loc):
        pins.append('pin out bit forced "1 while any %IX input force is armed";')
    pins += [f'pin out bit step_{s.lower()} "SFC step {s.lower()}.X";' for s in steps]
    pins += h.pins_after_steps
    pins += [f'pin in {_KNOB_TYPES[t]} knob_{n.lower()} = {_c_literal(p.init.get(n, "0"))} "VAR_INPUT {n.lower()}";'
             for n, t in knobs]
    for i in ax:
        pins += [f"pin out bit ax{i}_execute;", f"pin out bit ax{i}_halt;", f"pin out s32 ax{i}_mode;",
                 f"pin out float ax{i}_position;", f"pin out float ax{i}_velocity;",
                 f"pin out float ax{i}_acceleration;", f"pin out float ax{i}_deceleration;", f"pin out float ax{i}_jerk;",
                 f"pin in u32 ax{i}_command_id;", f"pin in u32 ax{i}_done_id;", f"pin in u32 ax{i}_aborted_id;",
                 f"pin in u32 ax{i}_failed_id;",
                 f"pin in bit ax{i}_busy;", f"pin in bit ax{i}_in_velocity;", f"pin in s32 ax{i}_error_id;"]
        pins += [f'pin out bit ax{i}_reset "{doc(f"ax{i}_reset", f"mc-axis.{i}.reset: held while MC_Reset.Execute")}";',
                 f'pin in s32 ax{i}_axis_error_id "{doc(f"ax{i}_axis_error_id", f"mc-axis.{i}.axis-error-id: MC_ReadAxisError")}";']
        if p.power:
            pins += [f'pin out bit ax{i}_power "{doc(f"ax{i}_power", "MC_Power.Enable while the program runs")}";',
                     f'pin in s32 ax{i}_state "mc-axis.{i}.state: MC_Power.Status (AxisState, 0 Disabled)";']
        if p.set_position:
            pins.append(f'pin out bit ax{i}_set_position "mc-axis.{i}.set-position: MC_SetPosition edge (mode 1 = relative)";')
    n_ax = max(ax, default=-1) + 1 or 1
    loc_decl = "".join(f"static {t} loc_{k}X{a}_{b}; {t} *__{k}X{a}_{b} = &loc_{k}X{a}_{b};\n" for t, k, a, b, _v in loc)
    loc_decl += "".join(f"static {t} loc_{k}{s}{a}_{b}; {t} *__{k}{s}{a}_{b} = &loc_{k}{s}{a}_{b};\n"
                        for t, k, s, a, b, _v in wloc)
    # A force arms only on a rising edge of -force while permit = 1 and dies in the component itself when permit
    # drops (a hardware E-STOP clears permit without telling the operator tool): a force still set after the
    # reset stays dead until a new edge — the force table can not outlive an E-STOP
    ins = [(a, b) for _t, k, a, b, _v in loc if k == "I"]
    loc_decl += "".join(f"static int frc_arm_{a}_{b}, frc_prev_{a}_{b};\n" for a, b in ins)
    loc_decl += h.c_vars
    drop = f"!{h.drop} && " if h.drop else ""
    c_in = "".join(f"    frc_arm_{a}_{b} = permit && {drop}ix_{a}_{b}_force && (frc_arm_{a}_{b} || !frc_prev_{a}_{b}); "
                   f"frc_prev_{a}_{b} = ix_{a}_{b}_force;\n"
                   f"    loc_IX{a}_{b} = frc_arm_{a}_{b} ? ix_{a}_{b}_fval : ix_{a}_{b}; ix_{a}_{b}_armed_set(frc_arm_{a}_{b});\n"
                   for a, b in ins)
    if ins:
        c_in += "    forced_set(" + " || ".join(f"frc_arm_{a}_{b}" for a, b in ins) + ");\n"
    c_in += "".join(f"    loc_I{s}{a}_{b} = i{s.lower()}_{a}_{b};\n" for _t, k, s, a, b, _v in wloc if k == "I")
    c_in += "".join(f"    RES__INST0.{n}.value = knob_{n.lower()};\n" for n, _t in knobs)
    c_in += h.c_in
    c_in += "".join(f"    iec_ax[{i}].cmd_id = ax{i}_command_id; iec_ax[{i}].done_id = ax{i}_done_id; "
                    f"iec_ax[{i}].aborted_id = ax{i}_aborted_id; iec_ax[{i}].failed_id = ax{i}_failed_id; iec_ax[{i}].busy = ax{i}_busy; "
                    f"iec_ax[{i}].in_vel = ax{i}_in_velocity; iec_ax[{i}].error_id = ax{i}_error_id;\n" for i in ax)
    if p.power:
        c_in += "".join(f"    iec_ax[{i}].state = ax{i}_state;\n" for i in ax)
    c_in += "".join(f"    iec_ax[{i}].axis_error_id = ax{i}_axis_error_id;\n" for i in ax)
    c_out = "".join(f"    iec_issue(&iec_ax[{i}]); ax{i}_execute_set(iec_ax[{i}].exec); ax{i}_halt_set(iec_ax[{i}].halt); "
                    f"ax{i}_mode_set(iec_ax[{i}].mode); ax{i}_position_set(iec_ax[{i}].pos); "
                    f"ax{i}_velocity_set(iec_ax[{i}].vel); ax{i}_acceleration_set(iec_ax[{i}].acc); "
                    f"ax{i}_deceleration_set(iec_ax[{i}].dec); ax{i}_jerk_set(iec_ax[{i}].jerk);\n" for i in ax)
    reset_or = f" || {h.axis_reset}" if h.axis_reset else ""
    c_out += "".join(f"    ax{i}_reset_set(iec_ax[{i}].reset{reset_or});\n" for i in ax)
    if p.power:   # a stopped program enables nothing (run = enable AND permit)
        c_out += "".join(f"    ax{i}_power_set(run && iec_ax[{i}].power);\n" for i in ax)
    if p.set_position:
        c_out += "".join(f"    ax{i}_set_position_set(iec_ax[{i}].set);\n" for i in ax)
    c_out += "".join(f"    qx_{a}_{b}_set(run ? loc_QX{a}_{b} : qx_{a}_{b}_safe);\n" for _t, k, a, b, _v in loc if k == "Q")
    c_out += "".join(f"    q{s.lower()}_{a}_{b}_set(run ? loc_Q{s}{a}_{b} : q{s.lower()}_{a}_{b}_safe);\n"
                     for _t, k, s, a, b, _v in wloc if k == "Q")
    c_out += "".join(f"    step_{s.lower()}_set(RES__INST0.{s}_X.value);\n" for s in steps)
    c_out += h.c_out
    # Res.c includes POUS.h/Cfg.h/POUS.c by name — inline them: one .comp per program, no per-program header
    # or extra .c (only the shared MATIEC runtime headers ride next to it)
    res = p.gen["Res.c"]
    for f in ("POUS.h", "Cfg.h", "POUS.c"):
        res = res.replace(f'#include "{f}"', f"/* {f} (iec2c) */\n" + p.gen[f])
    statics = "".join(f", {s}" for s in h.statics)
    c_init = f" {h.c_init}" if h.c_init else ""
    hold = f"run && !{h.hold}" if h.hold else "run"
    enable = "enable" if h.enable is None else h.enable
    notes = "".join(f"{n}\n" for n in h.notes)
    comp_text = f'''component {comp} "MATIEC node executor: program {p.name} (iec2c C) in {thread}";
{notes}license "GPL";
{chr(10).join(pins)}
function _;
;;
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <limits.h>
#include <float.h>
#include <rtapi_math.h>
#pragma GCC visibility push(hidden)
{res}
/* MATIEC leaves iec_lib_fmod (iec_std_lib.h) to the runtime: REAL_TO_*INT rounding, MOD of reals — without it
   any program converting a real to an integer does not link */
double iec_lib_fmod(double x, double y) {{ return fmod(x, y); }}
unsigned long long common_ticktime__ = {p.period_ns}ULL;
unsigned long greatest_tick_count__ = 0;
TIME __CURRENT_TIME;
IEC_UDINT iec_tick;
{loc_decl}
/* setpos: the pending command is MC_SetPosition (issued on `set`, mode 0/1 = absolute/relative) */
struct iec_axis {{ int pend, exec, halt_pend, halt, mode, resume, reset_pend, reset, reset_hold, setpos, set; IEC_REAL pos, vel, acc, dec, jerk;
                  IEC_UDINT *mv_owner, *halt_owner, *reset_owner;
                  unsigned cmd_id, done_id, aborted_id, failed_id; int busy, in_vel, error_id, axis_error_id, power, state; }};
static struct iec_axis iec_ax[{n_ax}];
/* one C entry for MC_* (mc_plcopen.st): kind 0 abs, 1 rel, 2 velocity, 3 halt, 4 set position (v <> 0: relative),
   5 reset (the reset pin is held with Execute until the axis leaves ErrorStop). acc/dec/jerk of a move are latched
   with pos/vel on the edge and go to mc-axis.N as given (0 = axis default there). *act: Active (P1) — the instance
   owns the axis's newest command and the axis is Busy with it.
   The instance's own outcome by its command number (mc_axis done-id / aborted-id / failed-id), latched in *st
   (P1 § 2.4.1): 0 idle, 1 busy, 2 done, 3 error, 4 aborted, 5 busy + InVelocity, 6 busy, InVelocity given;
   Done/Error/CommandAborted/InVelocity stay while Execute is held, reset by its falling edge, one cycle when
   Execute fell before the end */
void iec_mc(int a, int kind, int rise, int ex, IEC_REAL p, IEC_REAL v, IEC_REAL acc, IEC_REAL dec, IEC_REAL jerk,
            IEC_UDINT *id, IEC_USINT *st, IEC_BOOL *done, IEC_BOOL *busy, IEC_BOOL *err, IEC_WORD *eid, IEC_BOOL *inv,
            IEC_BOOL *abt, IEC_BOOL *act) {{
    struct iec_axis *x = a >= 0 && a < {n_ax} ? &iec_ax[a] : 0;
    int s = *st, pend, d, e, ab;
    if (rise) {{
        s = 1; *eid = 0;
        if (!x) {{ s = 3; *eid = 1; }}
        else if (kind == 5) {{ x->reset_pend = 1; x->reset_owner = id; }}
        else if (kind == 3) {{ x->halt_pend = 1; x->halt_owner = id; }}
        else {{ x->pend = 1; x->setpos = kind == 4; x->mode = kind == 4 ? v != 0 : kind; x->pos = p; x->vel = kind == 4 ? 0 : v;
               x->acc = acc; x->dec = dec; x->jerk = jerk; x->mv_owner = id; x->resume = 0; }}
        *id = 0;
    }} else if (!ex && (s == 2 || s == 3 || s == 4)) s = 0;
    else if (!ex && s == 5) s = 6;
    if (x && kind == 5 && x->reset_owner == id) x->reset_hold = ex;
    if (x && (s == 1 || s == 5 || s == 6)) {{
        pend = kind == 5 ? x->reset_owner == id && x->reset_pend
             : kind == 3 ? x->halt_owner == id && x->halt_pend
             : x->mv_owner == id && (x->pend || x->resume);   /* a held move waits to be re-issued: still Busy */
        d = *id && x->done_id == *id; e = *id && x->failed_id == *id; ab = *id && x->aborted_id == *id;
        if (pend) ;
        else if (e) {{ s = 3; *eid = (IEC_WORD)x->error_id; }}
        else if (d) s = 2;
        else if (kind == 5) {{ if (!*id || x->reset_owner != id || !x->reset) s = 0; }}   /* outside ErrorStop mc_axis answers nothing */
        else if (ab || !*id) s = 4;                    /* aborted, or overridden before it got a number */
        else if (kind == 2 && s == 1 && x->cmd_id == *id && x->in_vel) s = 5;
    }}
    *st = s; *busy = s == 1 || s == 5 || s == 6; *done = s == 2; *err = s == 3; *abt = s == 4; *inv = s == 5;
    *act = x && *busy && *id && x->cmd_id == *id && x->busy;
    if (s != 3) *eid = 0;
}}
int iec_axis_error(int a, IEC_WORD *aid) {{
    if (a < 0 || a >= {n_ax}) return 0;
    *aid = (IEC_WORD)iec_ax[a].axis_error_id; return 1;
}}
/* MC_Power: Enable -> axN-power, Status = axis not Disabled (AxisState 0) */
int iec_axis_power(int a, IEC_BOOL en, IEC_BOOL *on) {{
    if (a < 0 || a >= {n_ax}) return 0;
    iec_ax[a].power = en; *on = iec_ax[a].state != 0; return 1;
}}
/* mc_axis numbers every rising edge (execute/halt) as the next command-id; back-to-back edges get a low tick */
static void iec_issue(struct iec_axis *x) {{
    int idle = !x->exec && !x->halt && !x->reset && !x->set;
    if (x->pend && idle) {{ if (x->setpos) x->set = 1; else x->exec = 1; x->pend = 0; if (x->mv_owner) *x->mv_owner = x->cmd_id + 1; }}
    else if (x->halt_pend && idle) {{ x->halt = 1; x->halt_pend = 0; if (x->halt_owner) *x->halt_owner = x->cmd_id + 1; }}
    else if (x->reset_pend && idle) {{ x->reset = 1; x->reset_pend = 0; *x->reset_owner = x->cmd_id + 1; }}
    else {{ x->exec = 0; x->halt = 0; x->set = 0; x->reset = x->reset && x->reset_hold; }}
}}
{h.c_decl}static int iec_ready{statics};
#pragma GCC visibility pop
FUNCTION(_) {{
    if (!iec_ready) {{ RES_init__();{c_init} iec_ready = 1; }}
{h.c_pre}    int run = {enable} && permit;
{c_in}{h.c_pre_run}    if ({hold}) {{
        __CURRENT_TIME.tv_nsec += period;
        while (__CURRENT_TIME.tv_nsec >= 1000000000) {{ __CURRENT_TIME.tv_nsec -= 1000000000; __CURRENT_TIME.tv_sec++; }}
        iec_tick++;
        RES_run__(iec_tick);
    }}
{c_out}}}
'''
    hal = [f"loadrt {comp}", f"addf {inst} {thread}"]
    if h.permit_signal:
        hal.append(f"net {h.permit_signal} => {inst}.permit")
    hal += h.hal_after_load
    for i in ax:
        for src in ("execute", "halt", "mode", "position", "velocity", "acceleration", "deceleration", "jerk"):
            hal.append(f"net {comp}-ax{i}-{src} {inst}.ax{i}-{src} => mc-axis.{i}.{src}")
        for src in ("command-id", "done-id", "aborted-id", "failed-id", "busy", "in-velocity", "error-id"):
            hal.append(f"net {comp}-ax{i}-{src} mc-axis.{i}.{src} => {inst}.ax{i}-{src}")
        hal.append(f"net {comp}-ax{i}-reset {inst}.ax{i}-reset => mc-axis.{i}.reset")
        hal.append(f"net {comp}-ax{i}-axis-error-id mc-axis.{i}.axis-error-id => {inst}.ax{i}-axis-error-id")
        if h.axis_hal:
            hal += h.axis_hal(i)
        if p.power:
            hal.append(f"net {comp}-ax{i}-state mc-axis.{i}.state => {inst}.ax{i}-state")
        if p.set_position:
            hal.append(f"net {comp}-ax{i}-set-position {inst}.ax{i}-set-position => mc-axis.{i}.set-position")
    return Comp(
        comp=comp, files={f"{comp}.comp": comp_text, **p.hdr}, hal_lines=hal,
        inputs={f"%IX{a}.{b}": f"{inst}.ix-{a}-{b}" for _t, k, a, b, _v in loc if k == "I"},
        outputs={f"%QX{a}.{b}": f"{inst}.qx-{a}-{b}" for _t, k, a, b, _v in loc if k == "Q"},
        words={f"%{k}{s}{a}.{b}": f"{inst}.{k.lower()}{s.lower()}-{a}-{b}" for _t, k, s, a, b, _v in wloc if k != "M"},
        knobs={n.lower(): f"{inst}.knob-{n.lower().replace('_', '-')}" for n, _t in knobs},
        step_pins={s.lower(): f"{inst}.step-{s.lower().replace('_', '-')}" for s in steps})


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="iec2comp", description="IEC 61131-3 program (ST/SFC) -> HAL component "
                                 "(.comp for halcompile) via the MATIEC compiler iec2c")
    ap.add_argument("program", help="program.st (one PROGRAM; FUNCTION_BLOCKs may precede it)")
    ap.add_argument("--thread-period", type=int, required=True, metavar="NS",
                    help="period of the HAL thread in ns: TASK INTERVAL and the tick of timers")
    ap.add_argument("--thread", default=DEFAULT_THREAD, help="thread for addf (default %(default)s)")
    ap.add_argument("--axis", action="append", default=[], metavar="NAME=N",
                    help="AXIS_REF variable NAME of VAR_EXTERNAL is mc-axis.N (repeatable)")
    ap.add_argument("--name", help="component name (default iec_<program>)")
    ap.add_argument("-o", "--output", default=".", help="directory for <name>.comp, <name>.hal and the headers")
    a = ap.parse_args(argv)
    try:
        axes = {}
        for x in a.axis:
            n, _, v = x.partition("=")
            if not n or not v.isdigit():
                raise IecCompError(f"--axis {x}: NAME=N expected")
            axes[n] = int(v)
        st = Path(a.program).read_text(encoding="utf-8")
        p = compile_program(st, axes, a.thread_period)
        c = render(p, a.name or f"iec_{p.name.lower()}", a.thread)
    except (IecCompError, OSError) as e:
        print(f"iec2comp: {e}", file=sys.stderr)
        return 1
    os.makedirs(a.output, exist_ok=True)
    for f, t in c.files.items():
        Path(a.output, f).write_text(t)
    Path(a.output, f"{c.comp}.hal").write_text("\n".join(c.hal_lines) + "\n")
    comp = os.path.join(a.output, f"{c.comp}.comp")
    print(f"{comp}: halcompile --install {shlex.quote(comp)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
