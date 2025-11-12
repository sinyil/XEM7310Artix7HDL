-----------------------------------------------------------------------------------------------
-- Company: Columbia University
-- Engineers: Sinan Yilmaz, Adriaan Taal, Jaebin Choi
--
-- Create Date:    16:30:32 09/16/2016 
-- Design Name:    pdShank_2022
-- Module Name:    pdShank_2022
-- Project Name:   pdShank_2022
-- Target Devices: Artix-7 on Opal Kelly XEM7310
-- Tool versions:  Vivado 2022.1
--
-- Revision: [Sinan] 11:18:16 08/31/2022
--           - Rewritten for XEM7310, using 200 MHz clock and 1GiB DDR3 memory.
--
---- Additional Comments:
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

entity pdShank_v7310 is
    generic(
        --Opal Kelly user variables
        N : INTEGER := 20;
        BLOCK_SIZE : INTEGER := 256;		--in bytes!
        
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
            
        FIFO_SIZE_256b : INTEGER := 1023;
        FIFO_SIZE_128b : INTEGER := 2047;
        FIFO_SIZE_32b : INTEGER := 8184;
        FIFO_SIZE_16b : INTEGER := 16383;
    
        --DDR3 Stuff
        BURST_LEN               : INTEGER := 8
    );
    Port (
        --Opal Kelly Host input/output
        okUH :  in std_logic_vector( 4  downto 0  );
        okHU :  out std_logic_vector( 2  downto 0  );
        okUHU :  inout std_logic_vector( 31  downto 0  );
        okAA :  inout std_logic;
        OKreset : in std_logic;
        
        sys_clk_p :  in std_logic;
        sys_clk_n :  in std_logic;
    
        --Physical output to floating pins
        DREAD : out STD_LOGIC;
        DCLEAR : out STD_LOGIC;
        
        -- System reset
        --reset : in STD_LOGIC; --We can use ep00wire for reset signals. No need for a port
        
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
        
        -- DDR3 Memory Interface
        ddr3_dq      : inout STD_LOGIC_VECTOR(31 downto 0);
        ddr3_dqs_n   : inout STD_LOGIC_VECTOR(3 downto 0);
        ddr3_dqs_p   : inout STD_LOGIC_VECTOR(3 downto 0);
        ddr3_addr    : out   STD_LOGIC_VECTOR(14 downto 0);
        ddr3_ba      : out   STD_LOGIC_VECTOR(2 downto 0);
        ddr3_ras_n   : out   STD_LOGIC;
        ddr3_cas_n   : out   STD_LOGIC;
        ddr3_we_n    : out   STD_LOGIC;
        ddr3_reset_n : out   STD_LOGIC;
        ddr3_ck_p    : out   STD_LOGIC_VECTOR(0 downto 0);
        ddr3_ck_n    : out   STD_LOGIC_VECTOR(0 downto 0);
        ddr3_cke     : out   STD_LOGIC_VECTOR(0 downto 0);
        ddr3_dm      : out   STD_LOGIC_VECTOR(3 downto 0);
        ddr3_odt     : out   STD_LOGIC_VECTOR(0 downto 0)
        
    );
end entity;

architecture Behavioral of pdShank_v7310 is
    
    --Reset signals
	signal reset : STD_LOGIC;
	--signal fifo_reset : STD_LOGIC;
	--signal rst_cnt : unsigned(3 downto 0) := "0000";
	signal mig_reset: std_logic;
	signal mig_sys_rst_input: std_logic;
    
    --Opal kelly signals
	signal okClk : std_logic;
	signal okHE : std_logic_vector( 112  downto 0  );
	signal okEH : std_logic_vector( 64  downto 0  );
	signal okEHx : std_logic_vector( N*65-1  downto 0  );

	--Standard useful signals
	signal one : STD_LOGIC := '1';
	signal zero : STD_LOGIC:= '0';
	signal reset_value_16b : STD_LOGIC_VECTOR(15 downto 0) := "0000000000000000";
    signal reset_value_32b : STD_LOGIC_VECTOR(31 downto 0) := "00000000000000000000000000000000";
	
	--A. Endpoint connections:
--WIREIN
	signal ep00wire        : STD_LOGIC_VECTOR(31 downto 0)  := reset_value_32b;
	--hex0: resets
			--bit 0 : reset
			--bit 1 : unused --used to be drp_reset
			--bit 2 : ram reset
			--bit 3 : fsm_reset
	--hex1: datapath enable
			--bit 4 : unused --used to be RAM write_enable
			--bit 5 : unused --used to be RAM read_enable
			--bit 6 : unused --used to be calib_int
			--bit 7 : unused --used to be valid_int
	--hex2: FSM settings
			--bit 8 : fsm_enable
			--bit 9 : unused --used to be fsm_delay
			--bit 10 : unused
			--bit 11 : unused --clk_gen_rst
	--hex3: PLL settings
			--bit 12 : unused --used to be SPAD_ON_clk_sel mux
			--bit 13 : unused --used to be pll_clkinsel --separate bitfiles for int and ext clocks instead.
			--bit 14 : unused --used to be ce_spadout
			--bit 15 : unused --used to be ce_laserclk
	--hex4: IC settings
			--bit 16 : unused --used to be RSTB
			--bit 17 : unused --used to be SRESET
			--bit 18 : unused
			--bit 19 : unused
	
	signal ep01wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_global_rst_period
						
	signal ep02wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:31 : fsm_integration_period
			
	signal ep03wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:3 : fsm_data_wait_cycles (3 downto 0)
	
	signal ep04wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 28 downto 0 : user_memory_limit
			
	--signal ep05wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--
			
	--signal ep06wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--
			
	--signal ep07wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--
			
	--signal ep08wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--
			
	--signal ep09wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--
			
	signal ep10wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--all bits : fsm_numRows (9 downto 0)
			
	signal ep11wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_transfer_period
	
	signal ep12wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_sel_row_period
			
	signal ep13wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_count_row_period
	
	signal ep14wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_reset_row_period
			
	signal ep15wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:9 : fsm_count_reset_row_period
	
--WIREOUT
	signal ep20wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--LSB: fifo1_wr_data_count
			--MSB: fifo1_rd_data_count
			
	signal ep21wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--LSB: fifo2_wr_data_count
			--MSB: fifo2_rd_data_count
			
    signal ep22wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--LSB: output_fifo_wr_data_count
			--MSB: output_fifo_rd_data_count
			
	signal ep23wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bit 0 : write_phase_complete
			--bit 1 : read_phase_complete
			--bits 2:4 : mem_mode
			--bit 5 : memoryFull
			--bit 6 : po0_ep_read
			--bit 7 : pipe_out_ready
			--bits 8:27 : unused ----------
			--bits 28:31 : mem_state
			
	signal ep24wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bit 0 : clk_locked --used to be pll_locked
			--bit 1 : init_calib_complete --used to be c3_pll_lock
			--bit 2 : ce_clk_generated
			--bits 3:31 : output_fifo_wr_data_count
			
	signal ep25wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
	       --bits 0:28 : output_fifo_rd_data_count
	       		
	signal ep26wire        : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;
			--bits 0:29 : memoryCount (in Bytes) until 1 GiB
			
--TRIGGERIN
	--signal ep40TrigIn : STD_LOGIC_VECTOR(31 downto 0):= reset_value_32b;	
			--bit 0 : enable trigger for the pll programmer (drp_sen)
			
--TRIGGEROUT			
	signal ep60TrigOut : STD_LOGIC_VECTOR(31 downto 0) := reset_value_32b;	
			--pipe_out_ready
	
    -- First stage FIFO signals (FSM to intermediate) - 16-bit to 128-bit
    --signal fifo1_din : STD_LOGIC_VECTOR(15 downto 0);
    signal fifo1_wr_en : STD_LOGIC;
    signal fifo1_full : STD_LOGIC;
    signal fifo1_dout : STD_LOGIC_VECTOR(127 downto 0);
    signal fifo1_rd_en : STD_LOGIC;
    signal fifo1_empty : STD_LOGIC;
    signal fifo1_valid : STD_LOGIC;
    signal fifo1_rd_data_count : STD_LOGIC_VECTOR(FIFO_d_128b -1 downto 0);
    signal fifo1_wr_data_count : STD_LOGIC_VECTOR(FIFO_d_16b -1 downto 0);
    
    -- Second stage FIFO signals (intermediate to Memory) - 128-bit to 256-bit
    --signal fifo2_din : STD_LOGIC_VECTOR(127 downto 0);
    signal fifo2_wr_en : STD_LOGIC;
    signal fifo2_full : STD_LOGIC;
    signal fifo2_dout : STD_LOGIC_VECTOR(255 downto 0);
    signal fifo2_rd_en : STD_LOGIC;
    signal fifo2_empty : STD_LOGIC;
    signal fifo2_valid : STD_LOGIC;
    signal fifo2_rd_data_count : STD_LOGIC_VECTOR(FIFO_d_256b -1 downto 0);
    signal fifo2_wr_data_count : STD_LOGIC_VECTOR(FIFO_d_128b -1 downto 0);
    
    -- Memory controller signals
    signal app_addr : STD_LOGIC_VECTOR(28 downto 0);
    signal app_cmd : STD_LOGIC_VECTOR(2 downto 0);
    signal app_en : STD_LOGIC;
    signal app_wdf_data : STD_LOGIC_VECTOR(255 downto 0);
    signal app_wdf_end : STD_LOGIC;
    signal app_wdf_wren : STD_LOGIC;
    signal app_wdf_mask : STD_LOGIC_VECTOR(31 downto 0);
    signal app_rd_data : STD_LOGIC_VECTOR(255 downto 0);
    signal app_rd_data_end : STD_LOGIC;
    signal app_rd_data_valid : STD_LOGIC;
    signal app_rdy : STD_LOGIC;
    signal app_wdf_rdy : STD_LOGIC;
    signal device_temp : STD_LOGIC_VECTOR(11 downto 0);
    
    -- Robust Memory Controller Process for pdShank_v7310. First Write and then Read.
    constant BURST_UI_WORD_COUNT : integer := 1;  -- 256-bit UI, BL8 = 1 UI word per burst. 32-bit words.
    constant ADDRESS_INCREMENT   : integer := 8;  -- BL8 burst = 8 word addresses for each write/read.
    constant MAX_MEMORY_WORDS    : unsigned(28 downto 0) := to_unsigned(268435455, 29); -- 10^30 bytes = 1GiB divided by 4. (Each word is 4 bytes.) 
    --constant READ_THROTTLE_CYCLES    : integer := 4;                  -- Wait cycles between reads
    --signal throttle_counter : unsigned(6 downto 0) := (others => '0');  -- For read throttling
    
    -- Address management
    signal cmd_byte_addr_wr : unsigned(28 downto 0) := (others => '0');
    signal cmd_byte_addr_rd : unsigned(28 downto 0) := (others => '0');
    type mem_mode_type is (MODE_IDLE, MODE_WRITE, MODE_READ, MODE_COMPLETE);
    signal mem_mode : mem_mode_type := MODE_IDLE;
    signal user_memory_limit : unsigned(28 downto 0);    -- User-defined memory limit
    -- Command/Data tracking for write operations to keep track of both commands and data writes. We need both to be synced.
    signal cmd_write_count : unsigned(28 downto 0) := (others => '0');
    signal data_write_count : unsigned(28 downto 0) := (others => '0');
    -- FSM control. To enable it only when in WRITE mode
    signal fsm_enable_internal : std_logic := '0';
    -- Phase completion flags
    signal write_phase_complete : std_logic := '0';
    signal read_phase_complete : std_logic := '0';
    signal memoryCount      : unsigned(29 downto 0) := (others => '0'); -- Tracks total bytes written. Until 1GiB.
    signal memoryFull     : std_logic := '0';  -- Memory approaching full
    
    -- Memory state machine
    type mem_state_type is (
        s_idle,
        s_write_wait_data,
        s_write_send_data,
        s_write_wait_cmd_rdy,
        s_write_send_cmd,
        s_write_cmd_done,
        s_read_wait_rdy,
        s_read_send_cmd,
        s_read_wait_data,
        s_read_store_data,
        s_complete
    );
    signal mem_state : mem_state_type := s_idle;
    
    -- Output FIFO signals (Memory to PipeOut) - 256-bit to 32-bit
    signal output_fifo_din : STD_LOGIC_VECTOR(255 downto 0);
    signal output_fifo_wr_en : STD_LOGIC;
    signal output_fifo_full : STD_LOGIC;
    signal output_fifo_dout : STD_LOGIC_VECTOR(31 downto 0);
    signal output_fifo_rd_en : STD_LOGIC;
    signal output_fifo_empty : STD_LOGIC;
    signal output_fifo_rd_data_count : STD_LOGIC_VECTOR(16 -1 downto 0);
    signal output_fifo_wr_data_count : STD_LOGIC_VECTOR(13 -1 downto 0);
    
    signal pipe_out_ready :std_logic;
	signal po0_ep_read : std_logic;
    signal pipe_out_data : std_logic_vector(31 downto 0);
    
    -- Clock and reset signals
    signal clk_generated_intclk : std_logic; -- FSM clock from clock wizard
    signal clk_locked : STD_LOGIC;        -- Clock wizard locked
    signal ce_clk_generated	: std_logic;
    signal rst_clk_generated	: std_logic;
    signal rst_n : STD_LOGIC;
    signal ui_clk : STD_LOGIC;            -- MIG user interface clock
    signal ui_clk_sync_rst : STD_LOGIC;   -- MIG reset
    signal init_calib_complete : STD_LOGIC;
    
    -- FSM signals
    signal fsm_enable 		: STD_LOGIC;
    signal fsm_reset			: std_logic;
    signal fsm_data : STD_LOGIC_VECTOR(15 downto 0);
    
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
	
	signal fsm_dout_16b 			: std_logic_vector(15 downto 0) := reset_value_16b;
	signal fsm_rowCount	:	std_logic_vector( 9  downto 0  );
	signal fsm_frameCount	:	std_logic_vector( 9  downto 0  );
    
-- COMPONENT DECLARATIONS

    -- Clock Management for FSM clock generation
    component clk_wiz_0
        port (
            clk_out1 : out STD_LOGIC;
            locked : out STD_LOGIC;
            clk_in1 : in STD_LOGIC
        );
    end component;
    
    -- MIG Memory Controller
    component mig_XEM7310_400MHz
        port (
            ddr3_dq : inout STD_LOGIC_VECTOR(31 downto 0);
            ddr3_dqs_n : inout STD_LOGIC_VECTOR(3 downto 0);
            ddr3_dqs_p : inout STD_LOGIC_VECTOR(3 downto 0);
            ddr3_addr : out STD_LOGIC_VECTOR(14 downto 0);
            ddr3_ba : out STD_LOGIC_VECTOR(2 downto 0);
            ddr3_ras_n : out STD_LOGIC;
            ddr3_cas_n : out STD_LOGIC;
            ddr3_we_n : out STD_LOGIC;
            ddr3_reset_n : out STD_LOGIC;
            ddr3_ck_p : out STD_LOGIC_VECTOR(0 downto 0);
            ddr3_ck_n : out STD_LOGIC_VECTOR(0 downto 0);
            ddr3_cke : out STD_LOGIC_VECTOR(0 downto 0);
            ddr3_dm : out STD_LOGIC_VECTOR(3 downto 0);
            ddr3_odt : out STD_LOGIC_VECTOR(0 downto 0);
            sys_clk_p : in STD_LOGIC;
            sys_clk_n : in STD_LOGIC;
            app_addr : in STD_LOGIC_VECTOR(28 downto 0);
            app_cmd : in STD_LOGIC_VECTOR(2 downto 0);
            app_en : in STD_LOGIC;
            app_wdf_data : in STD_LOGIC_VECTOR(255 downto 0);
            app_wdf_end : in STD_LOGIC;
            app_wdf_mask : in STD_LOGIC_VECTOR(31 downto 0);
            app_wdf_wren : in STD_LOGIC;
            app_rd_data : out STD_LOGIC_VECTOR(255 downto 0);
            app_rd_data_end : out STD_LOGIC;
            app_rd_data_valid : out STD_LOGIC;
            app_rdy : out STD_LOGIC;
            app_wdf_rdy : out STD_LOGIC;
            app_sr_req : in STD_LOGIC;
            app_ref_req : in STD_LOGIC;
            app_zq_req : in STD_LOGIC;
            app_sr_active : out STD_LOGIC;
            app_ref_ack : out STD_LOGIC;
            app_zq_ack : out STD_LOGIC;
            ui_clk : out STD_LOGIC;
            ui_clk_sync_rst : out STD_LOGIC;
            init_calib_complete : out STD_LOGIC;
            device_temp : out STD_LOGIC_VECTOR(11 downto 0);
            sys_rst : in STD_LOGIC
        );
    end component;
    
    -- First Stage FIFO (16-bit to 128-bit)
    component FIFO_I16b_O128b is 
        port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector(15 downto 0);
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector(127 downto 0);
            full :  out std_logic;
            empty : out std_logic;
			valid : out std_logic;
            rd_data_count :  out std_logic_vector( FIFO_d_128b -1  downto 0 );
            wr_data_count :  out std_logic_vector( FIFO_d_16b -1  downto 0 )
        );
    end component;
    
    -- Second Stage FIFO (128-bit to 256-bit)
    component FIFO_I128b_O256b is 
        port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector(127 downto 0);
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector(255 downto 0);
            full :  out std_logic;
            empty : out std_logic;
			valid : out std_logic;
            rd_data_count :  out std_logic_vector( FIFO_d_256b -1  downto 0 );
            wr_data_count :  out std_logic_vector( FIFO_d_128b -1  downto 0 )
        );
    end component;
    
    -- Output FIFO (256-bit to 32-bit)
    component FIFO_I256b_O32b is 
        port (
            rst :  in std_logic;
            wr_clk :  in std_logic;
            rd_clk :  in std_logic;
            din :  in std_logic_vector(255  downto 0);
            wr_en :  in std_logic;
            rd_en :  in std_logic;
            dout :  out std_logic_vector(31 downto 0);
            full :  out std_logic;
            empty : out std_logic;
            rd_data_count :  out std_logic_vector( 16 -1  downto 0 ); -- larger than other FIFOs
            wr_data_count :  out std_logic_vector( 13 -1  downto 0 )
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
		rowCount : out std_logic_vector(9 downto 0); -- starts counting the row before reading, just after selecting it, not when done with reading.
		frameCount : out std_logic_vector(9 downto 0) -- counts the frame when the frame is done and state goes back to IDLE for the next frame.
		
		);
    end component;

begin

--A. Import all required data
--A1 Resets
	reset <= ep00wire(0);	--global reset
    rst_n <= not reset;
    --drp_reset <= ep00wire(1);
    mig_reset <= ( ep00wire(2) or ui_clk_sync_rst  );
    mig_sys_rst_input <= ep00wire(2);
    
--A3. FSM control signals
	fsm_reset <= ep00wire(3);
	fsm_enable <= ep00wire(8);   
    
    ce_clk_generated 		<= init_calib_complete and clk_locked; --important, only put out clk_generated_intclk when pll_locked and in record state! 	
    
    -- Internal FSM control - only enable during write mode
    fsm_enable_internal <= fsm_enable when mem_mode = MODE_WRITE else '0';
    
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
	fifo1_wr_en <= '1' when ((fsm_req_fifowr = '1') AND (fifo1_full = '0') AND (reset = '0') )
							 else '0';
    fifo1_rd_en <= '1' when ((fifo1_empty = '0') AND (fifo2_full = '0') AND (reset = '0') )
							 else '0';
	fifo2_wr_en <= '1' when ((fifo1_valid = '1') AND (fifo2_full = '0') AND (reset = '0') )
							 else '0';
	output_fifo_rd_en <= '1' when ((po0_ep_read = '1') AND (output_fifo_empty = '0') AND (reset = '0') )
							 else '0';
	pipe_out_ready <=  '1' when ((to_integer(unsigned(output_fifo_rd_data_count)) >= BLOCK_SIZE)  AND (reset = '0') )
						     else '0';
    
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
    
    ------------------------
    -- FSM Outputs added for pdShank_2022 ends here.
    ------------------------
    
    --uLEDs: the MSB (left) is the one closest to the USB cable
    led <= (not( init_calib_complete & clk_locked & write_phase_complete & read_phase_complete & fsm_enable_internal & fsm_reset & mig_reset & pipe_out_ready));
    
--C2. Pipe data. Somehow the bytes are intact but get reversed when they are put out via the pipe.
    pipe_out_data(15 downto 0) <= output_fifo_dout(31 downto 16);
    pipe_out_data(31 downto 16) <= output_fifo_dout(15 downto 0);
    --pipe_out_data <= fifo_o_data_out;

--C3. Endpoints
--    ep20wire(0) <= fsm_glob_tx;
--    ep20wire(1) <= fsm_glob_grst;
--    ep20wire(2) <= fsm_enRow_rst;
--    ep20wire(3) <= fsm_clk_sel;
--    ep20wire(4) <= fsm_clk_rst;
--    ep20wire(5) <= fsm_enRow_init;
--    ep20wire(6) <= fsm_ramp_swRst;
--    ep20wire(7) <= fsm_rst_ramp;
--    ep20wire(8) <= fsm_en_count;
--    ep20wire(10) <= fsm_rst_shift;
--    ep20wire(11) <= fsm_en_shift;
--    ep21wire(25 downto GPIO_IN_LEN) <= fsm_dout_16b; -- use the rest of it to get the data out of the chip
--    ep21wire(GPIO_IN_LEN-1 downto 0) <= DOUT; -- 10 bits
--    ep23wire(9 downto 0) <= fsm_rowCount;
--    ep23wire(19 downto 10) <= fsm_frameCount;
    
    ep20wire(FIFO_d_16b-1 downto 0) <= fifo1_wr_data_count;
    ep20wire(31 downto 31-FIFO_d_128b+1) <= fifo1_rd_data_count;
    ep21wire(FIFO_d_128b-1 downto 0) <= fifo2_wr_data_count;
    ep21wire(31 downto 31-FIFO_d_256b+1) <= fifo2_rd_data_count;
    ep22wire(13-1 downto 0) <= output_fifo_wr_data_count;
    ep22wire(31 downto 31-16+1) <= output_fifo_rd_data_count;
    
    ep23wire(0) <= write_phase_complete;
    ep23wire(1) <= read_phase_complete;
    ep23wire(4 downto 2) <= std_logic_vector(to_unsigned(mem_mode_type'pos(mem_mode), 3));
    ep23wire(5) <= memoryFull;
    ep23wire(6) <= po0_ep_read;
    ep23wire(7) <= pipe_out_ready;
    
    --ep23wire(27 downto 21) <= std_logic_vector(throttle_counter);
    ep23wire(31 downto 28) <= std_logic_vector(to_unsigned(mem_state_type'pos(mem_state), 4)); -- State machine
    
    ep24wire(0) <= clk_locked;
    ep24wire(1) <= init_calib_complete;
    ep24wire(2) <= ce_clk_generated;
    ep24wire(31 downto 3) <= std_logic_vector(cmd_byte_addr_wr);
    
    ep25wire(28 downto 0) <= std_logic_vector(cmd_byte_addr_rd);
    
    ep26wire(29 downto 0) <= std_logic_vector(memoryCount);
    
    clk_mgmt : clk_wiz_0
        port map (
            clk_out1 => clk_generated_intclk,     -- 200 MHz clock for FSM
            locked => clk_locked,
            clk_in1 => ui_clk
        );
    
    --clk_generated_intclk_n <= not(clk_generated_intclk);
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
    
    -- Memory Interface Generator (handles clocks internally)
    memory_controller : mig_XEM7310_400MHz
        port map (
            -- DDR3 Physical Interface
            ddr3_dq => ddr3_dq,
            ddr3_dqs_n => ddr3_dqs_n,
            ddr3_dqs_p => ddr3_dqs_p,
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
            
            -- System Interface (differential clock input)
            sys_clk_p => sys_clk_p,
            sys_clk_n => sys_clk_n,
            sys_rst => mig_sys_rst_input,
            
            -- Application Interface
            app_addr => app_addr,
            app_cmd => app_cmd,
            app_en => app_en,
            app_wdf_data => app_wdf_data,
            app_wdf_end => app_wdf_end,
            app_wdf_wren => app_wdf_wren,
            app_wdf_mask => app_wdf_mask,
            app_rd_data => app_rd_data,
            app_rd_data_end => app_rd_data_end,
            app_rd_data_valid => app_rd_data_valid,
            app_rdy => app_rdy,
            app_wdf_rdy => app_wdf_rdy,
            app_sr_req => '0',
            app_ref_req => '0',
            app_zq_req => '0',
            app_sr_active => open,
            app_ref_ack => open,
            app_zq_ack => open,
            
            -- Clock and Reset outputs
            ui_clk => ui_clk,
            ui_clk_sync_rst => ui_clk_sync_rst,
            init_calib_complete => init_calib_complete,
            device_temp => device_temp
        );
    
    -- First Stage FIFO (FSM to intermediate) -- Clock domain crossing
    FIFO_ib1 : FIFO_I16b_O128b
        port map (
            rst => not (ce_clk_generated and rst_n),
            wr_clk => clk_generated_intclk,           -- FSM clock domain
            rd_clk => ui_clk,               -- MIG clock domain
            din => fsm_dout_16b,
            wr_en => fifo1_wr_en,
            rd_en => fifo1_rd_en,
            dout => fifo1_dout,
            full => fifo1_full,
            empty => fifo1_empty,
            valid => fifo1_valid,
            rd_data_count => fifo1_rd_data_count,
            wr_data_count => fifo1_wr_data_count
        );
    
    -- Second Stage FIFO (intermediate to memory) -- Same clock domain
    FIFO_ib2 : FIFO_I128b_O256b
        port map (
            rst => not (ce_clk_generated and rst_n),
            wr_clk => ui_clk,               -- MIG clock domain
            rd_clk => ui_clk,               -- MIG clock domain
            din => fifo1_dout,
            wr_en => fifo2_wr_en,
            rd_en => fifo2_rd_en,
            dout => fifo2_dout,
            full => fifo2_full,
            empty => fifo2_empty,
            valid => fifo2_valid,
            rd_data_count => fifo2_rd_data_count,
            wr_data_count => fifo2_wr_data_count
        );
    
    -- Output FIFO (memory output to pipe out) -- Clock domain crossing
    FIFO_okPipeOut : FIFO_I256b_O32b
        port map (
            rst => not (ce_clk_generated and rst_n),
            wr_clk => ui_clk,               -- MIG clock domain
            rd_clk => okClk,           -- PipeOut clock domain
            din => output_fifo_din,
            wr_en => output_fifo_wr_en,
            rd_en => output_fifo_rd_en,
            dout => output_fifo_dout,
            full => output_fifo_full,
            empty => output_fifo_empty,
            rd_data_count => output_fifo_rd_data_count,
            wr_data_count => output_fifo_wr_data_count
        );
    
    -- FSM Instance (runs on 200MHz clock, only when in MEMORY WRITE mode)
    FSM_pdShank : FSM_pdShank_v4
	port map(
		clk_in => clk_generated_intclk, --clk_generated_intclk or clk_generated_extclk -- [sys_clk_ibufg = 100 MHz. or c3_clk0 = 40 MHz (measured)].
		fsm_en => fsm_enable_internal and ce_clk_generated,
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
    
    user_memory_limit <= unsigned(ep04wire(28 downto 0)); -- User sets memory usage limit
    
    memoryFull <= '1' when (cmd_byte_addr_wr >= user_memory_limit) or
                       (cmd_byte_addr_wr >= MAX_MEMORY_WORDS - ADDRESS_INCREMENT) else '0';
    
    -- Memory Mode Management Process
    mem_mode_manager : process(ui_clk)
    begin
        if rising_edge(ui_clk) then
            if ui_clk_sync_rst = '1' or mig_reset = '1' or init_calib_complete = '0' then
                mem_mode <= MODE_IDLE;
                write_phase_complete <= '0';
                read_phase_complete <= '0';
            else
                case mem_mode is
                
                    -- start with WRITE process whenever the clocks are locked and FSM is enabled.
                    when MODE_IDLE =>
                        write_phase_complete <= '0';
                        read_phase_complete <= '0';
                        if fsm_enable = '1' and ce_clk_generated = '1' then
                            mem_mode <= MODE_WRITE;
                        end if;
                    
                    -- the only way to get out of WRITE mode is reaching either user_memory_limit or 1GiB memory size
                    when MODE_WRITE =>
                        if cmd_byte_addr_wr >= user_memory_limit or 
                           cmd_byte_addr_wr >= (MAX_MEMORY_WORDS - ADDRESS_INCREMENT) then
                            mem_mode <= MODE_READ;
                            write_phase_complete <= '1';
                        end if;
                    
                    -- READ stops after we reach the address we left the WRITE process
                    when MODE_READ =>
                        if cmd_byte_addr_rd >= cmd_byte_addr_wr then
                            mem_mode <= MODE_COMPLETE;
                            read_phase_complete <= '1';
                        end if;
                        
                    when MODE_COMPLETE =>
                        null;
                        
                end case;
            end if;
        end if;
    end process;
    
    -- Memory Controller Process - following Opal Kelly pattern
    mem_ctrl_process : process(ui_clk)
    begin
        if rising_edge(ui_clk) then
            if ui_clk_sync_rst = '1' or init_calib_complete = '0' or mig_reset = '1' then
                mem_state <= s_idle;
                cmd_byte_addr_wr <= (others => '0');
                cmd_byte_addr_rd <= (others => '0');
                memoryCount <= (others => '0');
                cmd_write_count <= (others => '0');
                data_write_count <= (others => '0');
                
                app_en <= '0';
                app_cmd <= (others => '0');
                app_addr <= (others => '0');
                app_wdf_wren <= '0';
                app_wdf_end <= '0';
                app_wdf_data <= (others => '0');
                app_wdf_mask <= (others => '0');
                
                fifo2_rd_en <= '0';
                output_fifo_wr_en <= '0';
                output_fifo_din <= (others => '0');
                
            else
                -- Default: clear single-cycle signals
                app_en <= '0';
                app_wdf_wren <= '0';
                app_wdf_end <= '0';
                fifo2_rd_en <= '0';
                output_fifo_wr_en <= '0';
                
                case mem_state is
                    when s_idle =>
                        if mem_mode = MODE_WRITE and 
                           fifo2_empty = '0' and 
                           cmd_byte_addr_wr < user_memory_limit and
                           cmd_byte_addr_wr < (MAX_MEMORY_WORDS - ADDRESS_INCREMENT) then
                            if cmd_write_count = data_write_count then
                                mem_state <= s_write_wait_data;
                            end if;
                        elsif mem_mode = MODE_READ and
                              cmd_byte_addr_rd < cmd_byte_addr_wr and
                              output_fifo_full = '0' then
                            mem_state <= s_read_wait_rdy;
                        elsif mem_mode = MODE_COMPLETE then
                            mem_state <= s_complete;
                        end if;
                    
                    when s_write_wait_data =>
                        if fifo2_empty = '0' then
                            fifo2_rd_en <= '1';
                            mem_state <= s_write_send_data;
                        else
                            mem_state <= s_idle;
                        end if;
                    
                    when s_write_send_data =>
                        if fifo2_valid = '1' then
                            app_wdf_data <= fifo2_dout;
                            mem_state <= s_write_wait_cmd_rdy;
                        end if;
                    
                    when s_write_wait_cmd_rdy =>
                        if app_wdf_rdy = '1' then
                            app_wdf_wren <= '1';
                            app_wdf_end <= '1';
                            data_write_count <= data_write_count + 1;
                            mem_state <= s_write_send_cmd;
                        end if;
                    
                    when s_write_send_cmd =>
                        app_addr <= std_logic_vector(cmd_byte_addr_wr);
                        app_cmd <= "000";
                        app_en <= '1';
                        if app_rdy = '1' then
                            cmd_write_count <= cmd_write_count + 1;
                            mem_state <= s_write_cmd_done;
                        end if;
                    
                    when s_write_cmd_done =>
                        -- get the address ready for the next cycle and increment the memory by 32 bytes.
                        cmd_byte_addr_wr <= cmd_byte_addr_wr + ADDRESS_INCREMENT;
                        memoryCount <= memoryCount + 32;
                        mem_state <= s_idle;
                    
                    when s_read_wait_rdy =>
                        if app_rdy = '1' then
                            app_addr <= std_logic_vector(cmd_byte_addr_rd);
                            app_cmd <= "001";
                            app_en <= '1';
                            mem_state <= s_read_send_cmd;
                        end if;
                    
                    when s_read_send_cmd =>
                        if app_rdy = '1' then
                            cmd_byte_addr_rd <= cmd_byte_addr_rd + ADDRESS_INCREMENT;
                            mem_state <= s_read_wait_data;
                        else
                            app_addr <= std_logic_vector(cmd_byte_addr_rd);
                            app_cmd <= "001";
                            app_en <= '1';
                        end if;
                    
                    when s_read_wait_data =>
                        if app_rd_data_valid = '1' then
                            output_fifo_din <= app_rd_data;
                            output_fifo_wr_en <= '1';
                            mem_state <= s_read_store_data;
                        end if;
                    
                    when s_read_store_data =>
                        output_fifo_wr_en <= '0';
                        mem_state <= s_idle;
                    
                    when s_complete =>
                        null;
                        
                end case;
                
                if mig_reset = '1' then
                    cmd_byte_addr_wr <= (others => '0');
                    cmd_byte_addr_rd <= (others => '0');
                    memoryCount <= (others => '0');
                    cmd_write_count <= (others => '0');
                    data_write_count <= (others => '0');
                end if;
                
            end if;
        end if;
    end process;
    
    okHI : okHost   port map (okAA => okAA, okClk => okClk, okEH => okEH, okHE => okHE, okHU => okHU, okUH => okUH, okUHU => okUHU);
    wireOR : okWireOR generic map (N=>N) port map (okEH => okEH, okEHx => okEHx);
    wi00 : okWireIn 	  port map (ep_addr => x"00" ,     ep_dataout => ep00wire,       okHE => okHE);
    wi01 : okWireIn 	  port map (ep_addr => x"01" ,     ep_dataout => ep01wire,       okHE => okHE);
    wi02 : okWireIn 	  port map (ep_addr => x"02" ,     ep_dataout => ep02wire,       okHE => okHE);
    wi03 : okWireIn 	  port map (ep_addr => x"03" ,     ep_dataout => ep03wire,       okHE => okHE);
    wi04 : okWireIn 	  port map (ep_addr => x"04" ,     ep_dataout => ep04wire,       okHE => okHE);
    --wi05 : okWireIn 	  port map (ep_addr => x"05" ,     ep_dataout => ep05wire,       okHE => okHE);
    --wi06 : okWireIn 	  port map (ep_addr => x"06" ,     ep_dataout => ep06wire,       okHE => okHE);
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
    wo25 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 6*65-1 downto 5*65 ),  ep_addr=>x"25", ep_datain=>ep25wire);
    wo26 : okWireOut    port map (okHE=>okHE, okEH=>okEHx( 7*65-1 downto 6*65 ),  ep_addr=>x"26", ep_datain=>ep26wire);
    --ti40 : okTriggerIn  port map (okHE=>okHE, ep_clk =>okClk,                     ep_addr=>x"40", ep_trigger=>ep40TrigIn);
    to60 : okTriggerOut port map (okHE=>okHE, ep_clk =>okClk, okEH => okEHx(8*65-1  downto 7*65),  ep_addr=>x"60", ep_trigger=>ep60TrigOut);
    epA0 : okBTPipeOut  port map (okHE=>okHE, okEH=>okEHx( 9*65-1 downto 8*65 ),  ep_addr=>x"A0", 
                              ep_read=>po0_ep_read, ep_blockstrobe=>open, ep_datain=>pipe_out_data, ep_ready=>pipe_out_ready);								  
end Behavioral;
