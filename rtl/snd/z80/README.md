# Z80 sound CPU — vendored core and adapter

`ngp_z80.sv` is ours. Everything under `tv80/` is third party.

## Provenance

| | |
|---|---|
| Upstream | `https://github.com/hutch31/tv80` |
| Commit tested and vendored | `66a131c38d05ef58b3d8c4f1507a72e6e4aa5d65` ("Merge pull request #6 from hharte/for-upstream", 2026-05-11) |
| Licence | MIT — `tv80/LICENSE`, and a copy of the same grant heads every source file |
| Files taken | `rtl/core/tv80_core.v`, `tv80_alu.v`, `tv80_mcode.v`, `tv80_reg.v`, `LICENSE` |
| Files deliberately NOT taken | `tv80s.v`, `tv80n.v` (their strobe decode is not `ce`-gated — see below), `sd_zmem.v`, `sd_access64.v`, the UART, the `verif/` tree and the Chisel sources |

The vendored commit **contains** `59d343ed417c1be24a6d000f11d27153af164f0d`
("Fix exz80all failures: DAA parity, CPI/CPD X/Y flags", 2026-05-10), which is
the third-party fix `agents/research/21` §4.1 flagged. Verified with
`git merge-base --is-ancestor`.

SHA-256 of the files as taken from upstream, before the local patches below:

```
904c180ea8df7d2a407187c2277424b5c803afb2bf14ac358f24826f40e61774  tv80_core.v
5bcfbcb1dda9d8eca18bdd8dae6da95dda062d0285d5976c2640d3ee11ef5cc9  tv80_alu.v
78c869b94fa8e37eb5492ffa9f92bfe2c6d12faac548c1a035719bb994152be8  tv80_mcode.v
e38fa80510f044c7f298b56d58c0d7fec4857e6e39e0efdac3f889da95b5e68c  tv80_reg.v
f18e8f2663611c1e4711a090016a1afafed71c546af195c0c76d97f1e8e5dde5  LICENSE
```

## House style does not apply to `tv80/`

`AGENTS.md`'s tab-indent rule and the project's zero-waiver lint standard are
**not** applied to the vendored files. They keep upstream's spaces-for-indent
formatting verbatim so a future `git diff` against upstream stays readable, and
`sim/run_z80_gate.sh` reports their lint warnings instead of gating on them.
`ngp_z80.sv` is ours and is held to the full standard: it lints clean under
`verilator --lint-only -Wall` with zero waivers.

Two further deviations from house style, both inherent to the core and both
harmless on Cyclone V:

- **Asynchronous reset.** `tv80_core` resets on `negedge reset_n`. The rest of
  `rtl/` uses synchronous reset. `AGENTS.md` bans negedge-*clocked* blocks, not
  async reset terms, and the async form is what lets the core be held in reset
  while `ce` is low — which is exactly what the savestate restore path needs
  (research/18 §8.1).
- **No `initial` state.** Everything architectural is reset explicitly, so C5's
  "reset must not depend on `initial` blocks" holds.

## Build requirement: `TV80_REFRESH` must be defined

This is not optional and it is easy to miss. Upstream leaves the macro
undefined by default, and its own `verif/doc/test_plan.md` records the ifdef
path as untested. With it undefined:

- there is **no R register at all** — `LD A,R` returns 0 and `LD R,A` is a
  no-op;
- `rfsh_n` is tied high and no refresh cycle is generated;
- the I:R address never appears on the bus during M1 T3/T4.

`sim/run_z80_gate.sh` and the `ngpc_z80_bench` CMake target both pass
`+define+TV80_REFRESH`, and the gate measures the ifdef path (C4 passes).
**Any Quartus build must add it too** (`files.qip` / project Verilog macro).

## Local patches

Every change is additive and tagged `NGPC:` or `NGPC FIX` in the source. There
are three groups.

### 1. Instruction boundary and observability (`tv80_core.v`)

New outputs `instr_boundary`, `naive_boundary`, `prefix_active`, `int_taken`.

`instr_boundary` mirrors the exact condition under which tv80's own sequencer
writes `mcycle <= 7'b0000001` and pulls `m1_n` low, minus the DD/FD
displacement detour, **and qualified by `Prefix == 2'b00`**.

That last qualifier is the important measured finding, and it corrects
research/21 §8 item 2. tv80 executes a DD/FD/CB/ED prefix byte as its own
one-M-cycle "instruction" — which is *correct* bus behaviour, because it is why
the following opcode byte gets a proper 4 T M1 with refresh, exactly as the
real part does. The consequence is that `last_mcycle & last_tstate`, the signal
research/21 proposed using, fires **between the prefix and its opcode**.
Measured over the first 37.1 M instructions of ZEXDOC: 86,939 such firings,
against 0 for `instr_boundary`. tv80's own interrupt-acceptance logic uses
`Prefix == 2'b00` for precisely this reason.

`naive_boundary` is exported only so the gate can keep measuring that
difference. Nothing in the shipped design uses it.

### 2. Savestate tap (`tv80_core.v`, `tv80_reg.v`)

New ports `ss_addr[3:0] / ss_wdata[31:0] / ss_wren / ss_rdata[31:0]` on
`tv80_core`, and a fourth (savestate) port `AddrS / DISH / DISL / WES / DOSH /
DOSL` on `tv80_reg`.

Word layout — `ngp_z80` forwards `ss_addr` unchanged:

| Word | `[31:16]` | `[15:0]` |
|---|---|---|
| 0x1 | BC | AF |
| 0x2 | DE | AF' |
| 0x3 | HL | SP |
| 0x4 | IX | PC |
| 0x5 | BC' | {I, R} |
| 0x6 | DE' | {8'h0, IFF1, IFF2, IM[1:0], halt, Alternate, XY_State[1:0]} |
| 0x7 | HL' | 0 |
| 0x8 | IY | 0 |

One register-file pair per word, so the file needs **one** extra read port
rather than two.

**The savestate writes are deliberately not re-gated by `cen`.** The engine's
strobe is one `clk_sys` cycle while `cen` is one in sixteen (one in 128 at
clock gear 4). This is the bug shape that already cost this project two real
defects (`ngp_intc` dropping INTAD, `t900_seq` dropping savestate writes). The
write can never race a normal one, because the engine only writes while the
core is parked and `cen` is held low.

### 3. NMI acceptance timing — a real defect, fixed here

**Measured before the fix: tv80 takes 15 T states to accept an NMI. The Z80
manual says 11** [A: Z80UM p.19, and the p.13 figure "Nonmaskable Interrupt
Request Operation"]. Two independent causes, both fixed:

| # | File | Change |
|---|---|---|
| 1 | `tv80_core.v` | `Auto_Wait` was asserted for `NMICycle` as well as `IntCycle`. The two automatic wait states belong only to the **maskable** acknowledge cycle. Removing it takes the NMI M1 from 7 T to 5 T. |
| 2 | `tv80_mcode.v` | The two NMI push M cycles were declared `TStates = 3'b100` (4 T). A Z80 memory write cycle is 3 T [A: Z80UM p.11]. Changed to `3'b011`. |

After the fix the acceptance is 5 + 3 + 3 = 11 T and every other measurement is
unchanged. Neither edit is reachable without an interrupt, so neither can
affect ZEXDOC or ZEXALL.

Worth reporting upstream.

## What the gate measured

Run `sim/run_z80_gate.sh`. Summary of the first run, on the commit above:

- **C1 ZEXDOC / ZEXALL** — see `agents/CHANGES.md` and the gate log.
- **C2** — 40 opcodes covering every M-cycle shape: **0 T-state mismatches, 0
  M-cycle mismatches** against the Z80 manual. M1 is 4 T with `/RFSH` in T3 and
  T4; memory read and write are 3 T; both I/O cycles are 4 T with the automatic
  wait.
- **C3** — every item passes *after* the NMI fix above; before it, the NMI
  11 T sub-item failed at 15 T.
- **C4** — R implemented, bit 7 preserved, DD prefix increments it twice,
  `LD A,R` copies IFF2 into P/V.
- **C6** — instruction trace bit-identical with `ce` at 1-in-1 and 1-in-8.
- **C8** — freeze by removing `ce` is transparent; save / scramble / restore at
  an instruction boundary is transparent.
- **Register file** — indices are **0=BC 1=DE 2=HL 3=IX 4=BC' 5=DE' 6=HL'
  7=IY**. research/21's inferred order (BC/DE/HL/BC'/DE'/HL'/IX/IY) has the
  right *set* and the wrong *order*; it is corrected in that note.

## Strobe-width note (fidelity, not correctness)

`ngp_z80`'s strobes are registered, so `/MREQ` and `/RD` are exactly one T
state wide and sit inside the real part's window rather than spanning it (the
real Z80 holds them for about 1.5 T). Read data is latched on the T2→T3 edge,
which is the real part's own sampling point. Nothing in the NGP sound block
observes strobe width — the shared RAM is BRAM and the TO3 latch clear watches
`/IORQ & /WR` as a level — so this is recorded rather than fixed.
