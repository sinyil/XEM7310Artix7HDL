# XEM7310Artix7HDL
HDL custom modules and projects for the Artix-7 on an Opal Kelly XEM7310 

These components need to be generated in the IP catalog to synthesize the code:

1. clk_wiz_0
2. FIFO_I16b_O128b
3. FIFO_I128b_O32b
4. FIFO_I128b_O256b
5. FIFO_I256b_O32b
6. mig_XEM7310_400MHz

Two different versions of the DDR3 memory is used here:
1. First one reads the data intermittently with the memory write process although the write process is prioritized. (pdShank_v7310_v6dot2_FIFOs_symmetrical.vhd)
2. Second one writes the whole data first, and then starts reading the memory. (pdShank_v7310_.vhd)

Python code is written for the second case.
