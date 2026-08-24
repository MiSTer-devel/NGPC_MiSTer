//
// TV80 8-Bit Microprocessor Core
// Based on the VHDL T80 core by Daniel Wallner (jesus@opencores.org)
//
// Copyright (c) 2004 Guy Hutchison (ghutchis@opencores.org)
//
// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"), 
// to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included 
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// NGPC-MODIFIED.  See rtl/snd/z80/README.md for the upstream commit and the
// full patch list.  The only change here is the AddrS/DOSH/DOSL/WES savestate
// port below; nothing on the original port list or in the original always
// block moved.  Third-party file: upstream formatting is preserved verbatim,
// including its spaces-for-indent style, per the project's vendoring rule.
module tv80_reg (/*AUTOARG*/
  // Outputs
  DOBH, DOAL, DOCL, DOBL, DOCH, DOAH,
  // Inputs
  AddrC, AddrA, AddrB, DIH, DIL, clk, CEN, WEH, WEL,
  // NGPC: savestate port (read always, write only while the core is parked)
  AddrS, DISH, DISL, WES, DOSH, DOSL
  );
    input  [2:0] AddrC;
    output [7:0] DOBH;
    input  [2:0] AddrA;
    input  [2:0] AddrB;
    input  [7:0] DIH;
    output [7:0] DOAL;
    output [7:0] DOCL;
    input  [7:0] DIL;
    output [7:0] DOBL;
    output [7:0] DOCH;
    output [7:0] DOAH;
    input  clk, CEN, WEH, WEL;
    // NGPC: savestate port.
    input  [2:0] AddrS;
    input  [7:0] DISH;
    input  [7:0] DISL;
    input        WES;
    output [7:0] DOSH;
    output [7:0] DOSL;

  reg [7:0] RegsH [0:7];
  reg [7:0] RegsL [0:7];

  // NGPC: the savestate write is deliberately NOT gated by CEN.  The engine's
  // write strobe is one clk_sys cycle wide while CEN is one in sixteen (one in
  // 128 at clock gear 4), so re-gating it would drop the write.  It can never
  // collide with a normal write: the engine only writes while the core is
  // parked, which is exactly when CEN is held low.
  always @(posedge clk)
    begin
      if (WES)
        begin
          RegsH[AddrS] <= DISH;
          RegsL[AddrS] <= DISL;
        end
      else if (CEN)
        begin
          if (WEH) RegsH[AddrA] <= DIH;
          if (WEL) RegsL[AddrA] <= DIL;
        end
    end

  assign DOSH = RegsH[AddrS];
  assign DOSL = RegsL[AddrS];


  assign DOAH = RegsH[AddrA];
  assign DOAL = RegsL[AddrA];
  assign DOBH = RegsH[AddrB];
  assign DOBL = RegsL[AddrB];
  assign DOCH = RegsH[AddrC];
  assign DOCL = RegsL[AddrC];

  // break out ram bits for waveform debug
// synopsys translate_off
  wire [7:0] B = RegsH[0];
  wire [7:0] BP = RegsH[4];
  wire [7:0] C = RegsL[0];
  wire [7:0] CP = RegsL[4];
  wire [7:0] D = RegsH[1];
  wire [7:0] DP = RegsH[5];
  wire [7:0] E = RegsL[1];
  wire [7:0] EP = RegsL[5];
  wire [7:0] H = RegsH[2];
  wire [7:0] HP = RegsH[6];
  wire [7:0] L = RegsL[2];
  wire [7:0] LP = RegsL[6];

  wire [15:0] IX = { RegsH[3], RegsL[3] };
  wire [15:0] IY = { RegsH[7], RegsL[7] };
// synopsys translate_on
  
endmodule

