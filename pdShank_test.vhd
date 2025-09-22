-----------------------------------------------------------------------------------------------
-- Company: Columbia University
-- Engineers: Adriaan Taal, Jaebin Choi, Sinan Yilmaz
--
-- Create Date:    16:30:32 09/16/2016 
-- Design Name:    pdShank_2022
-- Module Name:    pdShank_2022
-- Project Name:   pdShank_2022
-- Target Devices: Artix-7 on Opal Kelly XEM7310
-- Tool versions:  Vivado 2022.1
--
-- Revision: [Sinan] 11:18:16 08/31/2022
--           - Converted for XEM7310, using 200 MHz onboard clock and DDR3 memory.
--
---- Additional Comments: 
-- Uses XEM7310's 200 MHz LVDS clock directly, removing PLL-based clock generation.
-- DDR3 memory controller generated via Vivado MIG for 32-bit interface.
-----------------------------------------------------------------------------------------------

-----Required libraries
library ieee;
library work;
library MIG_7Series;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;
use ieee.numeric_std.all;
use work.FRONTPANEL.all;
library UNISIM;
use UNISIM.VComponents.all;

----The entity------------------
entity pdShank is
generic(
	--Opal Kelly user variables
	N : INTEGER := 20;
	BLOCK_SIZE : INTEGER := 512;		--in bytes!
	
	--FIFO and Pipe stuff
	GPIO_IN_LEN :INTEGER := 10; -- number of bits coming out of the chip
	PIPE_LEN : INTEGER := 4; 		 	--in bytes! 32 bits.
	
	--laser stuff
	--LASER_CLK_PERIOD_NS : real := 12.500; 
	SYS_CLK_PERIOD_NS : real := 5.000; --nanosecond
	SYS_CLK_PERIOD_NS_STRING : string := "5.000";--nanosecond
	
	FIFO_d_256b : INTEGER := 10; -- pdShank800 is using this.
	FIFO_d_128b : INTEGER := 11; -- pdShank800 is using this.
	FIFO_d_32b : INTEGER := 13; -- pdShank800 is using this.
	FIFO_d_16b : INTEGER := 14; -- pdShank800 is using this.
	--FIFO_HALF_d_16b : INTEGER := 8192;
	--FIFO_d_8b : INTEGER := 15;
	--FIFO_d_4b : INTEGER := 16;
		
	FIFO_SIZE_256b : INTEGER := 1023;
	FIFO_SIZE_128b : INTEGER := 2047;
	FIFO_SIZE_32b : INTEGER := 8188;
	FIFO_SIZE_16b : INTEGER := 16383;
	--FIFO_SIZE_8b : INTEGER := 32767;
	--FIFO_SIZE_4b : INTEGER := 65535;

	--DDR3 Stuff
    BURST_LEN               : INTEGER := 8;
	--P0_WR_SIZE              : INTEGER := 63;
	
	C3_P0_MASK_SIZE         : INTEGER := 32;--16;
	C3_P0_DATA_PORT_SIZE 	: INTEGER := 256;--128;
	C3_P1_MASK_SIZE         : integer := 32;--16;
	C3_P1_DATA_PORT_SIZE    : integer := 256;--128;
	C3_MEMCLK_PERIOD        : integer := 5000; -- 200 MHz = 5 ns period -- Memory data transfer clock period.
	C3_RST_ACT_LOW          : integer := 0;  -- # = 1 for active low reset, -- # = 0 for active high reset.
	C3_INPUT_CLK_TYPE       : string := "DIFFERENTIAL";  -- input clock type DIFFERENTIAL or SINGLE_ENDED.
	C3_CALIB_SOFT_IP        : string := "TRUE";  -- # = TRUE, Enables the soft calibration logic, -- # = FALSE, Disables the soft calibration logic.
	C3_SIMULATION           : string := "FALSE";  -- # = TRUE, Simulating the design. Useful to reduce the simulation time, -- # = FALSE, Implementing the design.
	DEBUG_EN                : integer := 0;  -- # = 1, Enable debug signals/controls, --   = 0, Disable debug signals/controls.
	C3_MEM_ADDR_ORDER       : string := "ROW_BANK_COLUMN";  -- The order in which user address is provided to the memory controller, -- ROW_BANK_COLUMN or BANK_ROW_COLUMN.
	C3_NUM_DQ_PINS          : integer := 32;  -- External memory data width. -- 32-bit DDR3
	C3_MEM_ADDR_WIDTH       : integer := 15;  -- External memory address width. -- Adjusted for DDR3
	C3_MEM_BANKADDR_WIDTH   : integer := 3 ; -- External memory bank address width.
	C3_MEM_TWTR : integer := 7500 --Latencies in picosecond, unused for low memory clock speeds
);

port (
	--Opal Kelly Host input/output
	okUH :  in std_logic_vector( 4  downto 0  );
	okHU :  out std_logic_vector( 2  downto 0  );
	okUHU :  inout std_logic_vector( 31  downto 0  );
	okAA :  inout std_logic;
	sys_clk_p :  in std_logic;
	sys_clk_n :  in std_logic;

	--Physical output to floating pins
	DREAD : out STD_LOGIC;
	DCLEAR : out STD_LOGIC;

	--------------
	--added for pdshank2022
	--------------
	DOUT : in std_logic_vector(GPIO_IN_LEN-1 downto 0); -- For pdShank, (9 downto 0) -- Physical input from IC
	
	glob_tx	: out std_logic; -- global transfer gate signal
	glob_grst	: out std_logic; -- global photodiode reset with Vdd

	enRow_rst	: out std_logic; -- reset and start the row selection avalanche. Low active.
	clk_sel	: out std_logic; -- select rows. Period*(1-DutyCycle) = row_sel time.
	clk_rst	: out std_logic; -- reset rows. delayed version of clk_sel.
	enRow_init	: out std_logic; -- initialize the avalanche. High for one rising edge clk_sel.
	
	ramp_swRst	: out std_logic; -- reset for the ramp switch. Low active.
	
	clk_ramp	: out std_logic; -- clk for the counter. should be the same as clk_in
	rst_ramp	: out std_logic; -- reset the counter.
	en_count	: out std_logic; -- enable for the counter
	
	clk_shift	: out std_logic; -- clk for the shift register. Slower than the clk_in.
	rst_shift	: out std_logic; -- reset the shift register outputs
	en_shift	: out std_logic; -- enable the shift register
												
	--------------
	--added for pdshank2022 ends here.
	--------------
	
	--Other PCB control
	REF_OUT : out STD_LOGIC; --mini; to drive OBIS laser
	LASER_IN : in STD_LOGIC; --mini; let external clock drive FPGA

	--Physical output to the opal kelly PCB
	led :  out std_logic_vector( 7  downto 0  ); -- outputs of the led indicate the status.
		
	-- DDR3 RAM in/output 
    ddr3_dq : inout std_logic_vector(31 downto 0);
    ddr3_dqs_n : inout std_logic_vector(3 downto 0);
    ddr3_dqs_p : inout std_logic_vector(3 downto 0);
    ddr3_addr : out std_logic_vector(14 downto 0);
    ddr3_ba : out std_logic_vector(2 downto 0);
    ddr3_ras_n : out std_logic;
    ddr3_cas_n : out std_logic;
    ddr3_we_n : out std_logic;
    ddr3_reset_n : out std_logic;
    ddr3_ck_p : out std_logic_vector(0 downto 0);
    ddr3_ck_n : out std_logic_vector(0 downto 0);
    ddr3_cke : out std_logic_vector(0 downto 0);
    ddr3_dm : out std_logic_vector(3 downto 0);
    ddr3_odt : out std_logic_vector(0 downto 0)
		
 );
end entity; 

----The architecture
architecture Behavioral of pdShank is

	--Reset signals
	signal reset : STD_LOGIC;
	--signal fifo_reset : STD_LOGIC;
	signal rst_cnt : unsigned(3 downto 0) := "0000";
	signal mcb_reset: std_logic;
	
	--Opal kelly signals
	signal okClk : std_logic;
	signal okHE : std_logic_vector( 112  downto 0  );
	signal okEH : std_logic_vector( 64  downto 0  );
	signal okEHx : std_logic_vector( N*65-1  downto 0  );

	--Standard useful signals
	signal one : STD_LOGIC := '1';
	signal zero : STD_LOGIC:= '0';
	signal reset_value_4b  : STD_LOGIC_VECTOR(3 downto 0) := "0000";	
	signal reset_value_8b  : STD_LOGIC_VECTOR(7 downto 0) := "00000000";	
	signal reset_value_16b : STD_LOGIC_VECTOR(15 downto 0) := "0000000000000000";	
    signal reset_value_32b : STD_LOGIC_VECTOR(31 downto 0) := "00000000000000000000000000000000";	
	signal reset_value     : STD_LOGIC_VECTOR(31 downto 0) := "00000000000000000000000000000000";	
	signal reset_value_128b: STD_LOGIC_VECTOR(127 downto 0) := "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";	
	signal reset_value_256b: STD_LOGIC_VECTOR(127 downto 0) := "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";	
	
--A. Endpoint connections:
--WIREIN
	signal ep00wire        : STD_LOGIC_VECTOR(31 downto 0)  := reset_value;
	--hex0: resets
			--bit 0 : reset
			--bit 1 : drp_reset
			--bit 2 : ram reset
			--bit 3 : fsm_reset
	--hex1: datapath enable
			--bit 4 : RAM write_enable
			--bit 5 : RAM read_enable
			--bit 6 : calib_int
			--bit 7 : unused --used to be valid_int
	--hex2: FSM settings
			--bit 8 : fsm_enable
			--bit 9 : unused --used to be fsm_delay
			--bit 10 : unused
			--bit 11 : unused --clk_gen_rst
	--hex3: PLL settings
			--bit 12 : unused --used to be SPAD_ON_clk_sel mux
			--bit 13 : unused --used to be pll_clkinsel --separate bitfiles for int and ext clocks instead.
			--bit 14 : unused -- used to be ce_spadout
			--bit 15 : unused -- used to be ce_laserclk
	--hex4: IC settings
			--bit 16 : unused -- used to be RSTB
			--bit 17 : unused -- used to be SRESET
			--bit 18 : unused
			--bit 19 : unused
	
	signal ep01wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_global_rst_period
						
	signal ep02wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_integration_period
			
	signal ep03wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : fsm_data_wait_cycles (3 downto 0)
	
	signal ep04wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 7 downto 0 : PLL_division_0
			--bits 15 downto 8 : PLL division_1
			
	signal ep05wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : CLKOUT0 phase
			
	signal ep06wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : CLKOUT0 duty cycle
			
	signal ep07wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : CLKOUT1 phase
			
	signal ep08wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : CLKOUT1 duty cycle
			
	signal ep09wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--
			
	signal ep10wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : fsm_numRows (9 downto 0)
			
	signal ep11wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_transfer_period
	
	signal ep12wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_sel_row_period
			
	signal ep13wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_count_row_period
	
	signal ep14wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_reset_row_period
			
	signal ep15wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : fsm_count_reset_row_period
	
--WIREOUT
	signal ep20wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bit 0 : glob_tx
			--bit 1 : glob_grst
			--bit 2 : enRow_rst
			--bit 3 : clk_sel
			--bit 4 : clk_rst
			--bit 5 : enRow_init
			--bit 6 : ramp_swRst
			--clk_ramp
			--bit 7 : rst_ramp
			--bit 8 : en_count
			--bit 9 : clk_shift
			--bit 10 : rst_shift
			--bit 11 : en_shift
			
	signal ep21wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : DOUT
			--bits 10:25 : fsm_dout_16b
			--			fsm_dout_16b = [4b - pixCount] [1b - frameBeginFlag] [1b 0 (Spacer)] [10b - pixMeasuredData]
			
   signal ep22wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:10 : ib1_count
			--bit 11 : unused
			--bit 12 : fifo_ib1_full
			--bit 13 : fifo_ib1_wren
			--bit 14 : po0_ep_read
			--bit 15 : fifo_o_empty
			--bit 16 : fifo_o_rd_en
			--bit 17 : pipe_out_ready
			--bits 18:28 : ob_count
			
	signal ep23wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--bits 0:9 : rowCount
			--bits 10:19 : frameCount
			
	signal ep24wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--		some outputs for control. Some are connected to the LEDs.
			--bit 0 : pll_locked
			--bit 1 : c3_pll_lock
			--bit 2 : unused
			--bit 3 : ce_clk_generated
			--bit 4 : drp_srdy
			--bits 5:7 : unused
			--bits 8:20 : fifo_o_rd_count
			
	signal ep26wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value;
			--all bits : memorycount
			
--TRIGGERIN
	signal ep40TrigIn : STD_LOGIC_VECTOR(31 downto 0):= reset_value;	
			--bit 0 : enable trigger for the pll programmer (drp_sen)
			
--TRIGGEROUT			
	signal ep60TrigOut : STD_LOGIC_VECTOR(31 downto 0) := reset_value;	
			--pipe_out_ready
	
--B. 		Component signals    
--B1. 	RAM system signals
    -- MIG signals
    signal c3_sys_clk : std_logic;
    signal c3_calib_done : std_logic;
    signal calib_done : std_logic;
    signal c3_clk0 : std_logic;
    signal c3_rst0 : std_logic;
    signal c3_async_rst : std_logic;
    signal c3_sysclk_2x : std_logic;
    signal c3_sysclk_2x_180 : std_logic;
    signal c3_pll_ce_0 : std_logic;
    signal c3_pll_ce_90 : std_logic;
    signal c3_pll_lock : std_logic;
    signal c3_mcb_drp_clk : std_logic;
    --RAM Command signals
    signal c3_p0_cmd_clk : std_logic;
    signal c3_p0_cmd_en : std_logic;
    signal c3_p0_cmd_instr : std_logic_vector(2 downto 0);
    signal c3_p0_cmd_bl : std_logic_vector(5 downto 0);
    signal c3_p0_cmd_byte_addr : std_logic_vector(28 downto 0);
    signal c3_p0_cmd_empty : std_logic;
    signal c3_p0_cmd_full : std_logic;
    --RAM Read signals
    signal c3_p0_wr_clk : std_logic;
    signal c3_p0_wr_en : std_logic;
    signal c3_p0_wr_mask : std_logic_vector(C3_P0_MASK_SIZE-1 downto 0);
    signal c3_p0_wr_data : std_logic_vector(C3_P0_DATA_PORT_SIZE-1 downto 0);
    signal c3_p0_wr_full : std_logic;
    signal c3_p0_wr_empty : std_logic;
    signal c3_p0_wr_count : std_logic_vector(6 downto 0);
    signal c3_p0_wr_underrun : std_logic;
    signal c3_p0_wr_error : std_logic;
    --RAM Write signals
    signal c3_p0_rd_clk : std_logic;
    signal c3_p0_rd_en : std_logic;
    signal c3_p0_rd_data : std_logic_vector(C3_P0_DATA_PORT_SIZE-1 downto 0);
    signal c3_p0_rd_full : std_logic;
    signal c3_p0_rd_empty : std_logic;
    signal c3_p0_rd_count : std_logic_vector(6 downto 0);
    signal c3_p0_rd_overflow : std_logic;
    signal c3_p0_rd_error : std_logic;
    
    signal c3_p0_sr_req : std_logic;
    signal c3_p0_ref_req : std_logic;
    signal c3_p0_zq_req : std_logic;
    signal c3_p0_sr_active : std_logic;
    signal c3_p0_ref_ack : std_logic;
    signal c3_p0_zq_ack : std_logic;
    signal c3_p0_rd_data_end : std_logic;
    
	--B2 RAM controller signals
	signal ob_count: std_logic_vector( FIFO_d_256b - 1  downto 0  ); --in
	signal ob_data : std_logic_vector( C3_P0_DATA_PORT_SIZE - 1  downto 0  ); --in
	signal ob_write_en: std_logic;		--out
	signal ob_full: std_logic;		--out

	signal ib1_count: std_logic_vector( FIFO_d_128b - 1  downto 0  ); --in
	signal ib1_data: std_logic_vector( 127  downto 0  )  := reset_value_128b;	--in
	signal ib1_empty: std_logic;			--in
	signal ib1_valid: std_logic;			--in
	signal ib1_read_en: std_logic;		--out
	signal ib2_count: std_logic_vector( FIFO_d_256b - 1  downto 0  ); --in
	signal ib2_data: std_logic_vector( C3_P0_DATA_PORT_SIZE -1  downto 0  )  := reset_value_256b;	--in
	signal ib2_empty: std_logic;			--in
	signal ib2_valid: std_logic;			--in
	signal ib2_read_en: std_logic;		--out
	--status 
	signal memoryCount :  std_logic_vector(26 downto 0); 
	signal mcbstate :  std_logic_vector(3 downto 0); 

--B3. FSM signals
	signal clk_FSM_FIFO 		: std_logic;
	signal clk_FSM_FIFO_n 	: std_logic;
	
	signal fsm_enable 		: STD_LOGIC := '1';
	signal fsm_clk_generated_intclk_EN	: STD_LOGIC;
	signal fsm_clk_generated_extclk_EN	: STD_LOGIC;
	
	-- Clock signals
    signal clk_200mhz : std_logic;
    signal clk_generated_intclk : std_logic;
	signal clk_generated_extclk : std_logic;
	signal fsm_reset			: std_logic := '0';
	signal clk_generated_intclk_n : std_logic;
	signal rst_clk_generated	: std_logic;
	
	-------------------
	-- added for pdShank_2022
	-------------------
	signal fsm_global_rst_period	: std_logic_vector(9 downto 0);
	signal fsm_integration_period	: std_logic_vector(31 downto 0);
	signal fsm_transfer_period	: std_logic_vector(9 downto 0);
	signal fsm_sel_row_period	: std_logic_vector(9 downto 0);
	signal fsm_count_row_period	: std_logic_vector(9 downto 0);
	--signal fsm_read_row_period	: std_logic_vector(3 downto 0);
	signal fsm_reset_row_period		: std_logic_vector(9 downto 0);
	signal fsm_count_reset_row_period		: std_logic_vector(9 downto 0);
	--signal fsm_read_reset_row_period		: std_logic_vector(3 downto 0);
	
	signal fsm_data_wait_cycles : std_logic_vector(3 downto 0) := "0111";
	signal fsm_numRows : std_logic_vector(9 downto 0) := "1100100000";
	
	signal fsm_req_fifowr 	: STD_LOGIC;
	
	signal fsm_glob_tx 	: STD_LOGIC;
	signal fsm_glob_grst 	: STD_LOGIC;
	signal fsm_enRow_rst 	: STD_LOGIC;
	signal fsm_clk_sel 	: STD_LOGIC;
	signal fsm_clk_rst 	: STD_LOGIC;
	signal fsm_enRow_init 	: STD_LOGIC;
	signal fsm_ramp_swRst 	: STD_LOGIC;
	--signal fsm_clk_ramp	: STD_LOGIC;
	signal fsm_rst_ramp 	: STD_LOGIC;
	signal fsm_en_count 	: STD_LOGIC;
	signal fsm_clk_shift	: STD_LOGIC;
	signal fsm_rst_shift 	: STD_LOGIC;
	signal fsm_en_shift 	: STD_LOGIC;
	
	--signal fsm_clk_in_FSM_freqDivBy2	: STD_LOGIC;
	
	signal fsm_dout_16b 			: std_logic_vector(15 downto 0) := reset_value_16b;
	signal fsm_rowCount	:	std_logic_vector( 9  downto 0  );
	signal fsm_frameCount	:	std_logic_vector( 9  downto 0  );
	signal fsm_frameCountMaxFlag	:	std_logic;
	
	-- Clock Generator Signals
	signal clk_generated_1	:	STD_LOGIC;
	--signal clk_generated_2	:	STD_LOGIC;
	signal clk_gen_rst		:	STD_LOGIC := zero; -- input to reset the clock generator.
	--signal clk_gen_status	:	STD_LOGIC_VECTOR(2 downto 0);
	signal clk_gen_locked	:	STD_LOGIC;
	--signal clk_gen_clk_valid:	STD_LOGIC;
	signal ce_clk_generated	: std_logic;
	
	-------------------
	-- added for pdShank_2022 ends here.
	-------------------
	 
--B4. FIFO_IB signals
	signal fifo_ib1_wren : std_logic;
	signal fifo_ib1_full : std_logic;
	signal fifo_ib1_wr_count: std_logic_vector(FIFO_d_16b -1 downto 0);
	signal fifo_ib2_wren : std_logic;
	signal fifo_ib2_full : std_logic;
	signal fifo_ib2_wr_count: std_logic_vector(FIFO_d_128b -1 downto 0);
	 
--B5. FIFO_O & pipe signals
	signal pipe_out_ready :std_logic;
	signal po0_ep_read : std_logic;
	signal fifo_o_rd_count: std_logic_vector(FIFO_d_32b-1 downto 0);
	signal fifo_o_data_out: std_logic_vector(31 downto 0);
	signal pipe_out_data : std_logic_vector(31 downto 0);
	signal fifo_o_rd_en : std_logic;
	signal fifo_o_empty : std_logic;

--B6. FIFO_I and pipe in signals
	
--C. Custom signals
	--nothing here yet!
	signal dummydata : std_logic_vector (9 downto 0) := "0011110011";
	
----END OF SIGNAL DECLARATION-----------------------------------------------------------------------------

----BEGIN COMPONENT DECLARATION----------------------------------------------------------------------------

component FIFO_I256b_O32b is 
         port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector( C3_P0_DATA_PORT_SIZE -1  downto 0  );
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector( 31  downto 0  );
            full :  out std_logic;
            empty : out std_logic;
            rd_data_count :  out std_logic_vector( FIFO_d_32b -1  downto 0  );
            wr_data_count :  out std_logic_vector( FIFO_d_256b -1  downto 0  )
        );
end component;

component FIFO_I128b_O256b is 
         port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector( 127  downto 0  );
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector( C3_P0_DATA_PORT_SIZE -1  downto 0  );
            full :  out std_logic;
            empty : out std_logic;
			valid : out std_logic;
            rd_data_count :  out std_logic_vector( FIFO_d_256b -1  downto 0  );
            wr_data_count :  out std_logic_vector( FIFO_d_128b -1  downto 0  )
        );
end component;

component FIFO_I16b_O128b is 
         port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector( 15  downto 0  );
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector( 127  downto 0  );
            full :  out std_logic;
            empty : out std_logic;
			valid : out std_logic;
            rd_data_count :  out std_logic_vector( FIFO_d_128b -1  downto 0  );
            wr_data_count :  out std_logic_vector( FIFO_d_16b -1  downto 0  )
        );
end component;

component FSM_pdShank_v4 is port (
		clk_in	:	in std_logic;
		fsm_en	:	in std_logic;
		fsm_rst	:	in std_logic;
		
		--control signals to IC
		MEM_CLEAR	: out std_logic; --marks beginning of a frame
		READ_EN	: out std_logic;
		
		DIN	: in std_logic_vector(9 downto 0);		-- data from IC. 10b
		req_fifowr	: out std_logic; -- fifo write enable
		--------------
		glob_tx	: out std_logic; -- global transfer gate signal
		glob_grst	: out std_logic; -- global photodiode reset with Vdd

		enRow_rst	: out std_logic; -- reset and start the row selection avalanche. Low active.
		clk_sel	: out std_logic; -- select rows. Period*(1-DutyCycle) = row_sel time.
		clk_rst	: out std_logic; -- reset rows. delayed version of clk_sel.
		enRow_init	: out std_logic; -- initialize the avalanche. High for one rising edge clk_sel.
		
		ramp_swRst	: out std_logic; -- reset for the ramp switch. Low active.
		
		--clk_ramp	: out std_logic; -- clk for the counter. should be the same as clk_in
		rst_ramp	: out std_logic; -- reset the counter.
		en_count	: out std_logic; -- enable for the counter
		
		clk_shift	: out std_logic; -- clk for the shift register. Slower than the clk_in.
		rst_shift	: out std_logic; -- reset the shift register outputs
		en_shift	: out std_logic; -- enable the shift register
		
		--clk_in_FSM_freqDivBy2 : out std_logic; -- clk entered into FSM frequency divided by 2. For debugging purposes only.
		
		global_rst_period : in std_logic_vector(9 downto 0);
		integration_period : in std_logic_vector(31 downto 0);
		transfer_period : in std_logic_vector(9 downto 0);
		sel_row_period : in std_logic_vector(9 downto 0);
		count_row_period : in std_logic_vector(9 downto 0);
		--read_row_period
		reset_row_period : in std_logic_vector(9 downto 0);
		count_reset_row_period : in std_logic_vector(9 downto 0);
		--read_reset_row_period
		data_wait_cycles : in std_logic_vector(3 downto 0); --4b. wait cycles after adress update to settle chip memory to data output.
		numRows : in std_logic_vector(9 downto 0); --10b. pixelArray[0]. Number of rows read, maximum should be 800. pixelArray[1]==10, constant.
		
		dout : out std_logic_vector(15 downto 0); --data out. 16b. [4b-ADDRESS + 2b-00 + 10b-DIN]
		rowCount : out std_logic_vector(9 downto 0); -- starts counting the row before just after selecting it, not when done with reading.
		frameCount : out std_logic_vector(9 downto 0) -- counts the frame when the frame is done and state goes back to IDLE for the next frame.
		
		);
end component;

component mig_XEM7310_400MHz is
generic
  (
	C3_P0_MASK_SIZE         : integer;
	C3_P0_DATA_PORT_SIZE    : integer;
	C3_P1_MASK_SIZE         : integer;
	C3_P1_DATA_PORT_SIZE    : integer;
	C3_MEMCLK_PERIOD        : integer; -- Memory data transfer clock period.
	C3_RST_ACT_LOW          : integer; -- # = 1 for active low reset,
												  -- # = 0 for active high reset.
	C3_INPUT_CLK_TYPE       : string; -- input clock type DIFFERENTIAL or SINGLE_ENDED.
	C3_CALIB_SOFT_IP        : string; -- # = TRUE, Enables the soft calibration logic,
												 -- # = FALSE, Disables the soft calibration logic.
	C3_SIMULATION           : string; -- # = TRUE, Simulating the design. Useful to reduce the simulation time,
												 -- # = FALSE, Implementing the design.
	DEBUG_EN                : integer; -- # = 1, Enable debug signals/controls,
												  --   = 0, Disable debug signals/controls.
	C3_MEM_ADDR_ORDER       : string; -- The order in which user address is provided to the memory controller,
												 -- ROW_BANK_COLUMN or BANK_ROW_COLUMN.
	C3_NUM_DQ_PINS          : integer; -- External memory data width.
	C3_MEM_ADDR_WIDTH       : integer; -- External memory address width.
	C3_MEM_BANKADDR_WIDTH   : integer -- External memory bank address width.
);
   
  port (
    -- Physical DDR3 Interface
    ddr3_dq         : inout std_logic_vector(31 downto 0);
    ddr3_dqs_p      : inout std_logic_vector(3 downto 0);
    ddr3_dqs_n      : inout std_logic_vector(3 downto 0);
    ddr3_addr       : out   std_logic_vector(14 downto 0);
    ddr3_ba         : out   std_logic_vector(2 downto 0);
    ddr3_ras_n      : out   std_logic;
    ddr3_cas_n      : out   std_logic;
    ddr3_we_n       : out   std_logic;
    ddr3_reset_n    : out   std_logic;
    ddr3_ck_p       : out   std_logic_vector(0 downto 0);
    ddr3_ck_n       : out   std_logic_vector(0 downto 0);
    ddr3_cke        : out   std_logic_vector(0 downto 0);
    ddr3_dm         : out   std_logic_vector(3 downto 0);
    ddr3_odt        : out   std_logic_vector(0 downto 0);
    -- System Clock and Reset
    sys_clk_p       : in    std_logic;
    sys_clk_n       : in    std_logic;
    sys_rst         : in    std_logic;
    ui_clk          : out   std_logic;
    ui_clk_sync_rst : out   std_logic;
    --mmcm_locked     : out   std_logic;
    init_calib_complete : out std_logic;
    device_temp     : out   std_logic_vector(11 downto 0);
    -- Application User Interface
    app_addr        : in    std_logic_vector(28 downto 0);
    app_cmd         : in    std_logic_vector(2 downto 0);
    app_en          : in    std_logic;
    app_wdf_data    : in    std_logic_vector(255 downto 0);
    app_wdf_end     : in    std_logic;
    app_wdf_wren    : in    std_logic;
    app_wdf_mask    : in    std_logic_vector(31 downto 0);
    app_rd_data     : out   std_logic_vector(255 downto 0);
    app_rd_data_end : out   std_logic;
    app_rd_data_valid : out std_logic;
    app_rdy         : out   std_logic;
    app_wdf_rdy     : out   std_logic;
    app_sr_req      : in    std_logic;
    app_ref_req     : in    std_logic;
    app_zq_req      : in    std_logic;
    app_sr_active   : out   std_logic;
    app_ref_ack     : out   std_logic;
    app_zq_ack      : out   std_logic
    -- Conditional SKIP_CALIB ports (include only if needed)
    -- calib_tap_req      : out std_logic;
    -- calib_tap_load     : in  std_logic;
    -- calib_tap_addr     : in  std_logic_vector(6 downto 0);
    -- calib_tap_val      : in  std_logic_vector(7 downto 0);
    -- calib_tap_load_done : in  std_logic
  );
end component;

----END OF COMPONENT DECLARATION----------------------------------------------------------------------------

----EXECUTION----------------------------------------------------------------------------------------------
begin --Concurrent part

--A. Import all required data
	--A1 Resets
	reset <= ep00wire(0);	--global reset
	--drp_reset <= ep00wire(1);
	mcb_reset <= ( ep00wire(2) or c3_rst0 );
	--c3_sys_rst_i <= ep00wire(2);
	c3_async_rst <= ep00wire(2);
	
	--A2 Signals set for RAM functionality
    --ddr2_cs_n <= '0';  --chip select enable active low, has to be 0!
	--ddr2_ck <= ddr2_ck_sig;
	--ddr2_ck_n <= ddr2_ck_n_sig;
	--ddr3_ck_p <= c3_sysclk_2x;
	--ddr3_ck_n <= c3_sysclk_2x_180;
	calib_done <= c3_calib_done or ep00wire(6);
	
	--A3. FSM control signals
	fsm_reset <= ep00wire(3);
	fsm_enable 		<= ep00wire(8);
	--fsm_delay ddr2_cs_n		<= ep00wire(9);
	clk_gen_rst	<=	ep00wire(11); -- used by clock_generator. Not using in this design.
	
	ce_clk_generated 		<= c3_pll_lock; --and fsm_clk_generated_intclk_EN; --important, only put out clk_generated_intclk when pll_locked and in record state! 	
	
	fsm_global_rst_period <= ep01wire(9 downto 0);
	fsm_integration_period <= ep02wire(31 downto 0);
	fsm_transfer_period <= ep11wire(9 downto 0);
	fsm_sel_row_period <= ep12wire(9 downto 0);
	fsm_count_row_period <= ep13wire(9 downto 0);
	-----------------fsm_read_row_period
	fsm_reset_row_period <= ep14wire(9 downto 0);
	fsm_count_reset_row_period <= ep15wire(9 downto 0);
	-----------------fsm_read_reset_row_period
	
	fsm_data_wait_cycles <= ep03wire(3 downto 0);
	fsm_numRows <= ep10wire(9 downto 0);
	
	--B2. Datapath control logic
	--Data is written by the FSM, important to not write when full!
	fifo_ib1_wren <= '1' when ((fsm_req_fifowr = '1') AND (fifo_ib1_full = '0') AND (reset = '0') )
							 else '0';
	fifo_ib2_wren <= '1' when ((fsm_req_fifowr = '1') AND (fifo_ib2_full = '0') AND (reset = '0') )
							 else '0';			
	fifo_o_rd_en <= '1' when ((po0_ep_read = '1') AND (fifo_o_empty = '0') AND (reset = '0') )
							 else '0';	 
	pipe_out_ready <=  '1' when ((to_integer(unsigned(fifo_o_rd_count)) >= BLOCK_SIZE)  AND (reset = '0') )
						else '0';
	
	--B3. Clock signals

	--B4. Programmable PLL signals
	--drp_sen <= ep40TrigIn(0);   --enable comes before the ready

------------------------
-- FSM Outputs added for pdShank_2022
------------------------
glob_tx	<=	fsm_glob_tx; -- global transfer gate signal
glob_grst	<=	fsm_glob_grst; -- global photodiode reset with Vdd

enRow_rst	<=	fsm_enRow_rst; -- reset and start the row selection avalanche. Low active.
clk_sel	<=	fsm_clk_sel; -- select rows. Period*(1-DutyCycle) = row_sel time.
clk_rst	<=	fsm_clk_rst; -- reset rows. delayed version of clk_sel.
enRow_init	<=	fsm_enRow_init; -- initialize the avalanche. High for one rising edge clk_sel.

ramp_swRst	<=	fsm_ramp_swRst; -- reset for the ramp switch. Low active.

--clk_ramp	<=	clk_generated_1; -- clk for the counter. should be the same as clk_in. 156.25 MHz.
rst_ramp	<=	fsm_rst_ramp; -- reset the counter.
en_count	<=	fsm_en_count; -- enable for the counter

clk_shift	<=	fsm_clk_shift; -- clk for the shift register. Slower than the clk_in.
rst_shift	<=	fsm_rst_shift; -- reset the shift register outputs
en_shift	<=	fsm_en_shift; -- enable the shift register

--clk_in_FSM_freqDivBy2 <= fsm_clk_in_FSM_freqDivBy2;   -- clk entered into FSM frequency divided by 2. For debugging purposes only.

------------------------
-- FSM Outputs added for pdShank_2022 ends here.
------------------------

--the MSB (left) is the one closest to the USB cable
--led <= (not( pll_locked & ce_clk_generated & c3_pll_lock & ep00wire(17) &  mcbstate));
fsm_frameCountMaxFlag <= fsm_frameCount(9) and fsm_frameCount(8) and fsm_frameCount(7) and fsm_frameCount(6) and fsm_frameCount(5) and fsm_frameCount(4) and fsm_frameCount(3) and fsm_frameCount(2) and fsm_frameCount(1) and fsm_frameCount(0);
led <= (not( c3_pll_lock & fsm_frameCountMaxFlag & fsm_reset & mcb_reset & mcbstate)); -- mcbstate=4bits

--C2. Pipe data. Somehow the bytes are intact but get reversed when they are put out via the pipe.
pipe_out_data(15 downto 0) <= fifo_o_data_out(31 downto 16);
pipe_out_data(31 downto 16) <= fifo_o_data_out(15 downto 0);
--pipe_out_data <= fifo_o_data_out;

--C3. Endpoints
ep20wire(0) <= fsm_glob_tx;
ep20wire(1) <= fsm_glob_grst;
ep20wire(2) <= fsm_enRow_rst;
ep20wire(3) <= fsm_clk_sel;
ep20wire(4) <= fsm_clk_rst;
ep20wire(5) <= fsm_enRow_init;
ep20wire(6) <= fsm_ramp_swRst;

ep20wire(7) <= fsm_rst_ramp;
ep20wire(8) <= fsm_en_count;

ep20wire(10) <= fsm_rst_shift;
ep20wire(11) <= fsm_en_shift;

--ep20wire(12) <= fsm_clk_in_FSM_freqDivBy2;

--ep20wire(13) <= clk_generated_1;

ep21wire(25 downto GPIO_IN_LEN) <= fsm_dout_16b; -- use the rest of it to get the data out of the chip
ep21wire(GPIO_IN_LEN-1 downto 0) <= DOUT; -- 10 bits

ep22wire(FIFO_d_128b-1 downto 0) <= ib1_count;

ep22wire(12) <= fifo_ib1_full;
ep22wire(13) <= fifo_ib1_wren;
	
ep22wire(14) <= po0_ep_read;
ep22wire(15) <= fifo_o_empty;
ep22wire(16) <= fifo_o_rd_en;
	
ep22wire(17) <= pipe_out_ready;
	
ep22wire(28 downto 18) <= ob_count;

ep23wire(9 downto 0) <= fsm_rowCount;
ep23wire(19 downto 10) <= fsm_frameCount;

----if we had used clock_generator, use here.
--ep24wire(0) <= clk_gen_rst;
--ep24wire(3 downto 1) <= clk_gen_status;
--ep24wire(4) <= clk_gen_locked;
--ep24wire(5) <= clk_gen_clk_valid;

--ep24wire(0) <= pll_locked;
ep24wire(1) <= c3_pll_lock;
ep24wire(3) <= ce_clk_generated;
--ep24wire(4) <= drp_srdy;
ep24wire(FIFO_d_32b+8-1 downto 8) <= fifo_o_rd_count;

ep26wire(26 downto 0) <= memoryCount;

--Start port maps

-- Clock input buffer for 200 MHz LVDS clock
    sysclk_ibufds : IBUFDS
    generic map (
        IOSTANDARD => "LVDS"
    )
    port map (
        I => sys_clk_p,
        IB => sys_clk_n,
        O => clk_200mhz
    );
    
    -- Global clock buffer
    clk_bufg : BUFG
    port map (
        I => clk_200mhz,
        O => clk_generated_intclk
    );
    
    clk_generated_intclk_n <= not(clk_generated_intclk);
    rst_clk_generated <= not(ce_clk_generated);
    
    --This kind of buffer is necessary to forward the clk_ramp on the output
    clkrampbuffer : ODDR generic map(
        DDR_CLK_EDGE => "SAME_EDGE", --Or OPPOSITE_EDGE for 180° phase
        INIT => '0', -- Sets initial state of the Q output to '0' or '1'
        SRTYPE => "SYNC" -- Specifies "SYNC" or "ASYNC" set/reset
        )
     port map (
        Q  => clk_ramp,    -- 1-bit output data
        C => clk_generated_intclk,    -- 1-bit clock input (unlike ODDR2's C0/C1)
        CE => ce_clk_generated,   -- 1-bit clock enable input
        D1 => '1',   -- 1-bit data input (Rising edge data)
        D2 => '0',   -- 1-bit data input (Falling edge data)
        R  => rst_clk_generated,   -- 1-bit reset input
        S  => '0'    -- 1-bit set input
    );
    
    --This kind of buffer is necessary to forward the REF_OUT on the output
    refoutbuffer : ODDR generic map(
        DDR_CLK_EDGE => "SAME_EDGE", --Or OPPOSITE_EDGE for 180° phase
        INIT => '0', -- Sets initial state of the Q output to '0' or '1'
        SRTYPE => "SYNC" -- Specifies "SYNC" or "ASYNC" set/reset
        )
     port map (
        Q  => REF_OUT,    -- 1-bit output data
        C => clk_generated_intclk,    -- 1-bit clock input (unlike ODDR2's C0/C1)
        CE => ce_clk_generated,   -- 1-bit clock enable input
        D1 => '1',   -- 1-bit data input (Rising edge data)
        D2 => '0',   -- 1-bit data input (Falling edge data)
        R  => rst_clk_generated,   -- 1-bit reset input
        S  => '0'    -- 1-bit set input
    );
    
-- MIG DDR3 Controller (Generated by Vivado MIG)
MIG: mig_XEM7310_400MHz generic map ( C3_P0_MASK_SIZE    => C3_P0_MASK_SIZE,
		C3_P0_DATA_PORT_SIZE   => C3_P0_DATA_PORT_SIZE,
		C3_P1_MASK_SIZE       => C3_P1_MASK_SIZE,
		C3_P1_DATA_PORT_SIZE   => C3_P1_DATA_PORT_SIZE,
		C3_MEMCLK_PERIOD      => C3_MEMCLK_PERIOD,
		C3_RST_ACT_LOW     =>C3_RST_ACT_LOW,
		C3_INPUT_CLK_TYPE=>C3_INPUT_CLK_TYPE,
		C3_CALIB_SOFT_IP   => C3_CALIB_SOFT_IP,
		C3_SIMULATION      =>C3_SIMULATION,
		DEBUG_EN    =>DEBUG_EN                ,
		C3_MEM_ADDR_ORDER=>C3_MEM_ADDR_ORDER,
		C3_NUM_DQ_PINS    =>C3_NUM_DQ_PINS,
		C3_MEM_ADDR_WIDTH  => C3_MEM_ADDR_WIDTH,
		C3_MEM_BANKADDR_WIDTH   =>C3_MEM_BANKADDR_WIDTH
		)
    
	port map(--These signals come directly from the DDR GPIO
		ddr3_dq => ddr3_dq,
        ddr3_dqs_p => ddr3_dqs_p,
        ddr3_dqs_n => ddr3_dqs_n,
        ddr3_addr => ddr3_addr,
        ddr3_ba => ddr3_ba,
        ddr3_ras_n => ddr3_ras_n,
        ddr3_cas_n => ddr3_cas_n,
        ddr3_we_n => ddr3_we_n,
        ddr3_reset_n => ddr3_reset_n,
        ddr3_ck_p => ddr3_ck_p,
        ddr3_ck_n => ddr3_ck_n,
        ddr3_cke => ddr3_cke,
        ddr3_dm => ddr3_dm,
        ddr3_odt => ddr3_odt,
        sys_clk_p => sys_clk_p,
        sys_clk_n => sys_clk_n,
        sys_rst => mcb_reset,
        ui_clk => c3_clk0,
        ui_clk_sync_rst => c3_rst0,
        --mmcm_locked => c3_pll_lock,
        app_wdf_data => c3_p0_wr_data,
        app_wdf_end => c3_p0_wr_en,
        app_wdf_wren => c3_p0_wr_en,
        app_sr_req => c3_p0_sr_req,
        app_ref_req => c3_p0_ref_req,
        app_zq_req => c3_p0_zq_req,
        app_wdf_mask => c3_p0_wr_mask,
        app_addr => c3_p0_cmd_byte_addr,
        app_cmd => c3_p0_cmd_instr,
        app_en => c3_p0_cmd_en,
        app_rdy => c3_p0_cmd_full,
        app_wdf_rdy => c3_p0_wr_full,
        app_sr_active => c3_p0_sr_active,
        app_ref_ack => c3_p0_ref_ack,
        app_zq_ack => c3_p0_zq_ack,
        app_rd_data => c3_p0_rd_data,
        app_rd_data_end => c3_p0_rd_data_end,
        app_rd_data_valid => c3_p0_rd_en,
        init_calib_complete => c3_calib_done
		);

okHI : okHost   port map (okAA => okAA, okClk => okClk, okEH => okEH, okHE => okHE, okHU => okHU, okUH => okUH, okUHU => okUHU);
wireOR : okWireOR generic map (N=>N) port map (okEH => okEH, okEHx => okEHx);
wi00 : okWireIn 	  port map (ep_addr => x"00" ,     ep_dataout => ep00wire,       okHE => okHE);
wi01 : okWireIn 	  port map (ep_addr => x"01" ,     ep_dataout => ep01wire,       okHE => okHE);
wi02 : okWireIn 	  port map (ep_addr => x"02" ,     ep_dataout => ep02wire,       okHE => okHE);
wi03 : okWireIn 	  port map (ep_addr => x"03" ,     ep_dataout => ep03wire,       okHE => okHE);
wi04 : okWireIn 	  port map (ep_addr => x"04" ,     ep_dataout => ep04wire,       okHE => okHE);
wi05 : okWireIn 	  port map (ep_addr => x"05" ,     ep_dataout => ep05wire,       okHE => okHE);
wi06 : okWireIn 	  port map (ep_addr => x"06" ,     ep_dataout => ep06wire,       okHE => okHE);
--wi07 : okWireIn 	  port map (ep_addr => x"07" ,     ep_dataout => ep07wire,       okHE => okHE);
--wi08 : okWireIn 	  port map (ep_addr => x"08" ,     ep_dataout => ep08wire,       okHE => okHE);
--wi09 : okWireIn 	  port map (ep_addr => x"09" ,     ep_dataout => ep09wire,       okHE => okHE);
wi10 : okWireIn 	  port map (ep_addr => x"10" ,     ep_dataout => ep10wire,       okHE => okHE);
wi11 : okWireIn 	  port map (ep_addr => x"11" ,     ep_dataout => ep11wire,       okHE => okHE);
wi12 : okWireIn 	  port map (ep_addr => x"12" ,     ep_dataout => ep12wire,       okHE => okHE);
wi13 : okWireIn 	  port map (ep_addr => x"13" ,     ep_dataout => ep13wire,       okHE => okHE);
wi14 : okWireIn 	  port map (ep_addr => x"14" ,     ep_dataout => ep14wire,       okHE => okHE);
wi15 : okWireIn 	  port map (ep_addr => x"15" ,     ep_dataout => ep15wire,       okHE => okHE);
wo20 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 1*65-1 downto 0*65 ),  ep_addr=>x"20", ep_datain=>ep20wire);
wo21 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 2*65-1 downto 1*65 ),  ep_addr=>x"21", ep_datain=>ep21wire);
wo22 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 3*65-1 downto 2*65 ),  ep_addr=>x"22", ep_datain=>ep22wire);
wo23 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 4*65-1 downto 3*65 ),  ep_addr=>x"23", ep_datain=>ep23wire);
wo24 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 5*65-1 downto 4*65 ),  ep_addr=>x"24", ep_datain=>ep24wire);
wo26 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 6*65-1 downto 5*65 ),  ep_addr=>x"26", ep_datain=>ep26wire);
ti40 : okTriggerIn  port map (okHE=>okHE, ep_clk =>okClk,                     ep_addr=>x"40", ep_trigger=>ep40TrigIn);
to60 : okTriggerOut port map (okHE=>okHE, ep_clk =>okClk, okEH => okEHx(7*65-1  downto 6*65),  ep_addr=>x"60", ep_trigger=>ep60TrigOut);
epA0 : okBTPipeOut  port map (okHE=>okHE, okEH=>okEHx( 8*65-1 downto 7*65 ),  ep_addr=>x"A0", 
                          ep_read=>po0_ep_read, ep_blockstrobe=>open, ep_datain=>pipe_out_data, ep_ready=>pipe_out_ready);								  

--This PLL feeds in these clock signals:
			--1). the internal 200MHz sys_clk_ibufg clock signal
			--2). -UNUSED-the external 80 MHz signal or external 20MHz signal
--It then puts out:
			--0). clk_in for FSM (and clk_ramp & REF_OUT to the PCB)

--lasertrigbuf : IBUFG port map(
--	i => LASER_IN,
--	O => LASER_TRIG_buf);
--
--laserclkbufio2 : BUFIO2 
--	  generic map(
--	  DIVIDE => 1,
--	  USE_DOUBLER => FALSE)
--	  port map(
--     I => LASER_TRIG_buf,
--	  DIVCLK => pll_laser_trig);
	
FIFO_okPipeOut : FIFO_I256b_O32b
	port map (
		rst => reset,							--in
		wr_clk => c3_clk0, 		--in				--synchronize with memory @200 MHZ
		rd_clk => okClk, 						--in	 			--synchronize with the Pipe fpga/host interface
		din => ob_data,						--in	 --connects to the RAM
		wr_en => ob_write_en,  				--in	 --the RAM decides when to write
		rd_en => fifo_o_rd_en, 				--in	 --connects to the Pipe
		dout => fifo_o_data_out,				--out	 --connects to the Pipe
		full => ob_full,						--out	
		empty => fifo_o_empty,				--out
		rd_data_count => fifo_o_rd_count,	--out 	--enables the pipe through a threshold
		wr_data_count => ob_count			--out		--synchronize with memory
	 );
	
FIFO_ib2 : FIFO_I128b_O256b
	port map(
		rst => reset, 
		wr_clk => clk_generated_intclk,			 --Internal FSM and FIFO clk
		rd_clk => c3_clk0, 				 --synchronize with the RAM @200 MHZ
		din 	 => ib1_data,	 			 --
		wr_en  => fifo_ib2_wren,			 --enable fifo when requested but not full
		rd_en  => ib2_read_en,			 --the RAM decides when to read from the FIFO
		dout   => ib2_data,				 --
		full   => fifo_ib2_full,			 --dont write when full!
		empty  => ib2_empty,				 --Required but not used in the MCBcontroller
		valid  => ib2_valid,				 --Required by MCBcontroller
		rd_data_count => ib2_count, 	 --Required by MCBcontroller
		wr_data_count => fifo_ib2_wr_count
		);
	
FIFO_ib1 : FIFO_I16b_O128b
	port map(
		rst => reset, 
		wr_clk => clk_generated_intclk,			 --Internal FSM and FIFO clk
		rd_clk => clk_generated_intclk, 				 --synchronize with the RAM @200 MHZ
		din 	 => fsm_dout_16b,	 			 --
		wr_en  => fifo_ib1_wren,			 --enable fifo when requested but not full
		rd_en  => ib1_read_en,			 --the RAM decides when to read from the FIFO
		dout   => ib1_data,				 --
		full   => fifo_ib1_full,			 --dont write when full!
		empty  => ib1_empty,				 --Required but not used in the MCBcontroller
		valid  => ib1_valid,				 --Required by MCBcontroller
		rd_data_count => ib1_count, 	 --Required by MCBcontroller
		wr_data_count => fifo_ib1_wr_count
		);

FSM_pdShank : FSM_pdShank_v4
	port map(
		clk_in => clk_generated_intclk, --clk_generated_intclk or clk_generated_extclk -- [sys_clk_ibufg = 100 MHz. or c3_clk0 = 40 MHz (measured)].
		fsm_en => fsm_enable,
		fsm_rst => fsm_reset,
		--control signals to IC
		MEM_CLEAR => DCLEAR, --marks beginning of a frame
		READ_EN => DREAD,
		--------------
		DIN => DOUT,
		req_fifowr => fsm_req_fifowr,
		--------------
		glob_tx => fsm_glob_tx, -- global transfer gate signal
		glob_grst => fsm_glob_grst, -- global photodiode reset with Vdd

		enRow_rst => fsm_enRow_rst, -- reset and start the row selection avalanche. Low active.
		clk_sel => fsm_clk_sel, -- select rows. Period*(1-DutyCycle) = row_sel time.
		clk_rst => fsm_clk_rst, -- reset rows. delayed version of clk_sel.
		enRow_init => fsm_enRow_init, -- initialize the avalanche. High for one rising edge clk_sel.
		
		ramp_swRst => fsm_ramp_swRst, -- reset for the ramp switch. Low active.
		
		--clk_ramp => clk_ramp, -- clk for the counter. should be the same as clk_in
		rst_ramp => fsm_rst_ramp, -- reset the counter.
		en_count => fsm_en_count, -- enable for the counter
		
		clk_shift => fsm_clk_shift, -- clk for the shift register. Slower than the clk_in?
		rst_shift => fsm_rst_shift, -- reset the shift register outputs
		en_shift => fsm_en_shift, -- enable the shift register
		
		--clk_in_FSM_freqDivBy2 => fsm_clk_in_FSM_freqDivBy2, -- clk entered into FSM frequency divided by 2. For debugging purposes only.
	 
		global_rst_period => fsm_global_rst_period,
		integration_period => fsm_integration_period,
		transfer_period => fsm_transfer_period,
		sel_row_period => fsm_sel_row_period,
		count_row_period => fsm_count_row_period,
		--read_row_period => fsm_read_row_period,
		reset_row_period => fsm_reset_row_period,
		count_reset_row_period => fsm_count_reset_row_period,
		--read_reset_row_period => fsm_read_reset_row_period,
		
		data_wait_cycles => fsm_data_wait_cycles,
		numRows => fsm_numRows,
		
		dout => fsm_dout_16b,
		rowCount => fsm_rowCount, -- starts counting the row before just after selecting it, not when done with reading.
		frameCount => fsm_frameCount -- counts the frame when the frame is done and state goes back to IDLE for the next frame.

	);
	
end Behavioral;
