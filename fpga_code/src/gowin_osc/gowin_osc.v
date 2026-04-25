//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.11.03 Education
//Part Number: GW1N-UV1P5QN48XFC7/I6
//Device: GW1N-1P5
//Device Version: C
//Created Time: Fri Apr 24 14:34:35 2026

module Gowin_OSC (oscout, oscen);

output oscout;
input oscen;

OSCO osc_inst (
    .OSCOUT(oscout),
    .OSCEN(oscen)
);

defparam osc_inst.FREQ_DIV = 8;
defparam osc_inst.REGULATOR_EN = 1'b0;

endmodule //Gowin_OSC
