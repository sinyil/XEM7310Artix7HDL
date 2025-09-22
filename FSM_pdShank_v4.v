`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Columbia University
// Engineer: Sinan Yilmaz
// 
// Create Date:    12:15:49 07/14/2022 
// Design Name: 
// Module Name:    FSM_pdShank_v1 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision:
// 	- v1 [sinan]: Created the module with all steps and flexible inputs.
//  	- v2 [sinan]: Added clk_shift. One less data_wait per row, but last column pixReading is strange.
//		- v3 [sinan]: --
//		- v4 [sinan]: Back to v2. + control over the number of rows read --> pixelArray[0].
//
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module FSM_pdShank_v4(
    clk_in, // Adjustable clock from the FPGA. Frequency, phase, duty cycle. Used as clk_ramp as well.
	 fsm_en, // fsm enable
	 fsm_rst, // fsm reset
	 //--------------
	 MEM_CLEAR, // marks beginning of a frame
	 READ_EN,
	 //--------------
	 DIN,					// data from IC. 10b
	 req_fifowr,			// FIFO flag. fifo write enable. 1b
    //--------------
	 glob_tx, // global transfer gate signal
    glob_grst, // global photodiode reset with Vdd

	 enRow_rst, // reset and start the row selection avalanche. Low active.
	 clk_sel, // select rows. Period*(1-DutyCycle) = row_sel time.
    clk_rst, // reset rows. delayed version of clk_sel.
    enRow_init, // initialize the avalanche. High for one rising edge clk_sel.
    
	 ramp_swRst, // reset for the ramp switch. Low active.
    
	 //clk_ramp, // clk for the counter. should be the same as clk_in
    rst_ramp, // reset the counter.
    en_count, // enable for the counter
	 
    clk_shift, // clk for the shift register. Slower than the clk_in.
    rst_shift, // reset the shift register outputs
    en_shift, // enable the shift register
	 
	 //clk_in_FSM_freqDivBy2, // clk entered into FSM frequency divided by 2. For debugging purposes only.
	 
	 global_rst_period,
	 integration_period,
	 transfer_period,
	 sel_row_period,
	 count_row_period,
	 //read_row_period,
	 reset_row_period,
	 count_reset_row_period,
	 //read_reset_row_period,
	 
	 data_wait_cycles, //4b. wait cycles after adress update to settle chip memory to data output.
	 numRows, //10b. pixelArray[0]. Number of rows read, maximum should be 800. pixelArray[1]==10, constant.
	 
	 dout,	// data out. 16b. [4b-ADDRESS + 2b-00 + 10b-DIN]
	 rowCount, // starts counting the row before just after selecting it, not when done with reading.
	 frameCount // counts the frame when the frame is done and state goes back to IDLE for the next frame.
    );

// to IC
output reg glob_tx;
output reg glob_grst;
output reg clk_sel;
output reg clk_rst;
output reg enRow_rst;
output reg enRow_init;
output reg ramp_swRst;
//	output reg clk_ramp;
output reg rst_ramp;
output reg en_count;
output reg clk_shift;
output reg rst_shift;
output reg en_shift;
//output reg clk_in_FSM_freqDivBy2;

output reg MEM_CLEAR;
output reg READ_EN;

output reg [15:0] dout;
output reg req_fifowr;

output reg [9:0] rowCount; // maximum is 800 or 400 depending on the chip type. two different bit files created.
output reg [9:0] frameCount;

// from FPGA
input wire clk_in;
input wire fsm_rst;
input wire fsm_en;
input wire [9:0] DIN;
input wire [9:0] global_rst_period;
input wire [31:0] integration_period;
input wire [9:0] transfer_period;
input wire [9:0] sel_row_period;
input wire [9:0] count_row_period;
//input wire [3:0] read_row_period = 4'b1010;
input wire [9:0] reset_row_period;
input wire [9:0] count_reset_row_period;
//input wire [3:0] read_reset_row_period = 4'b1010;

input wire [3:0] data_wait_cycles;
input wire [9:0] numRows;

//states
parameter IDLE = 4'b0000;
parameter CLEAR = 4'b0001;
parameter GLOBAL_RESET = 4'b0010;
parameter INTEGRATION = 4'b0011;
parameter TRANSFER = 4'b0100;
parameter SELECT_ROW = 4'b0101;
parameter COUNT_ROW = 4'b0110;
parameter READ_ROW = 4'b0111;
parameter RESET_ROW = 4'b1000;
parameter COUNT_RESET_ROW = 4'b1001;
parameter READ_RESET_ROW = 4'b1010;

//internals
reg [9:0] global_rst_period_cnt = 10'b0;
reg [31:0] integration_period_cnt = 32'b0;
reg [9:0] transfer_period_cnt = 10'b0;
reg [9:0] sel_row_period_cnt = 10'b0;
reg [9:0] count_row_period_cnt = 10'b0;
reg [9:0] read_row_period_cnt = 4'b0; // used to count pixel iteration in a row. Up to 10.
reg [9:0] reset_row_period_cnt = 10'b0;
reg [9:0] count_reset_row_period_cnt = 10'b0;
reg [9:0] read_reset_row_period_cnt = 4'b0; // used to count pixel iteration in a row. Up to 10.

reg [3:0] data_wait_cnt = 4'b0; // To wait for the data to settle before sent out.
reg [1:0] fifo_delay_cnt = 2'b0;

reg [1:0] clk_in_cnt = 2'b0;

//state internal
reg [3:0] state;

//concurrent logic

//sequential logic
always @(posedge clk_in or posedge fsm_rst) begin
	
	if(fsm_rst) begin
		//outputs
		glob_tx	<= 1'b0;
		glob_grst	<= 1'b0;
		clk_sel	<= 1'b0;
		clk_rst	<= 1'b0;
		enRow_rst	<= 1'b0;
		enRow_init	<= 1'b0;
		ramp_swRst	<= 1'b0;
		//clk_ramp	<= 1'b0;
		rst_ramp	<= 1'b0;
		en_count	<= 1'b0;
		clk_shift	<= 1'b0;
		rst_shift	<= 1'b0;
		en_shift	<= 1'b0;
		
		//clk_in_FSM_freqDivBy2 <= 1'b0;
		
		READ_EN <= 1'b0;
		MEM_CLEAR <= 1'b0;
		req_fifowr <= 1'b0;
		dout <= 16'b0;
		//internals
		rowCount	<= 10'b0;
		frameCount <= 10'b0;
		//state
		state <= IDLE;
	end
	
	else begin
		
		// to get the clk_in as an output frequency divided by 2. For debugging purposes.
		//clk_in_cnt <= clk_in_cnt + 1;
		//if(clk_in_cnt >= 1) begin
		//	clk_in_FSM_freqDivBy2 <= 1'b1;
		//	clk_in_cnt <= 2'b0;
		//end else begin
		//	clk_in_FSM_freqDivBy2 <= 1'b0;
		//end
		
		case(state)
			IDLE: begin
				//outputs
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b0;
				clk_sel	<= 1'b0;
				clk_rst	<= 1'b0;
				enRow_rst	<= 1'b0; // Low active
				enRow_init	<= 1'b0;
				ramp_swRst	<= 1'b0; // Low active
				//clk_ramp	<= 1'b0;
				rst_ramp	<= 1'b0;
				en_count	<= 1'b0;
				clk_shift	<= 1'b0;
				rst_shift	<= 1'b0;
				en_shift	<= 1'b0;
				
				READ_EN <= 1'b1;
				MEM_CLEAR <= 1'b0;
				req_fifowr <= 1'b0;
				dout <= 16'b0;
				//internals
				rowCount <= 10'b0;
				//state
				if(fsm_en) begin
					state <= CLEAR;
				end
			end
		
			CLEAR: begin //clear mem, marks beginning of a FRAME
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b0;
				clk_sel	<= 1'b0;
				clk_rst	<= 1'b0;
				enRow_rst	<= 1'b0;
				enRow_init	<= 1'b0;
				ramp_swRst	<= 1'b0;
				//clk_ramp	<= 1'b0;
				rst_ramp	<= 1'b0;
				en_count	<= 1'b0;
				clk_shift	<= 1'b0;
				rst_shift	<= 1'b0;
				en_shift	<= 1'b0;
				
				READ_EN <= 1'b1;
				MEM_CLEAR <= 1'b1; //assert mem_clear to chip
				req_fifowr <= 1'b0;
				dout <= 16'b0;
				//internals
				rowCount <= 10'b0;
				//state
				if(fsm_en && (rowCount == 0)) begin	
					state <= GLOBAL_RESET;		//start or continue the loop
				end else begin
					state <= CLEAR; 		//stay in clear
				end
			end
			
			GLOBAL_RESET: begin // global pixel photodiode reset.
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b1;
				enRow_rst	<= 1'b0; // it was already 0. Just to clarify. See explanation below.
				clk_sel	<= 1'b1; // these two are necessary to clear all AND inputs in row shifter. (shiftrow_v2)
				
				READ_EN <= 1'b1;
				MEM_CLEAR <= 1'b0;			//deassert mem_clear
				req_fifowr <= 1'b0;
				dout <= 16'b0;
				//internals				
				rowCount <= 10'b0;
				
				//state
				global_rst_period_cnt <= global_rst_period_cnt + 1;
				if((global_rst_period_cnt >= global_rst_period)) begin
					state <= INTEGRATION;
					global_rst_period_cnt <= 0;
				end else begin
					state <= GLOBAL_RESET;
				end
			end
			
			INTEGRATION: begin //integration of photons on the photodiode.
				//outputs
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b0;
				clk_sel	<= 1'b0;
				
				//internals
				rowCount <= 10'b0;
				
				//state
				integration_period_cnt <= integration_period_cnt + 1;
				if((integration_period_cnt >= integration_period)) begin
					state <= TRANSFER;
					integration_period_cnt <= 0;
				end else begin
					state <= INTEGRATION;
				end
			end
			
			TRANSFER: begin // transfer gate open for global shutter operation.
				//outputs
				glob_tx	<= 1'b1;
				glob_grst	<= 1'b0;
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				enRow_init	<= 1'b1; // starts the row avalanche. Needs a clk_sel rising and falling edge within.
				
				//internals
				rowCount <= 10'b0;
				
				//state
				transfer_period_cnt <= transfer_period_cnt + 1;
				if((transfer_period_cnt >= transfer_period)) begin
					state <= SELECT_ROW;
					transfer_period_cnt <= 0;
				end else begin
					state <= TRANSFER;
				end
			end
			
			SELECT_ROW: begin // 
				//outputs
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b0;
				clk_sel <= 1'b1; // we need a pulse of clk_sel here.
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				rst_ramp	<= 1'b1; // reset the counter.
				
				//internals
				
				if((rowCount >= numRows)) begin // if frame is complete, go to IDLE to start the new frame.
				// 10'b1100100000 == 800
				// 10'b0110010000 == 400
				// 10'b0100000000 == 256
				// 10'b0010000000 == 128
					state <= IDLE;
					frameCount <= frameCount + 1;
					rowCount <= 10'b0;
					
				end else begin
					//state
					sel_row_period_cnt <= sel_row_period_cnt + 1;
					if((sel_row_period_cnt >= sel_row_period)) begin
						clk_sel <= 1'b0; // pulse completed.
						rst_ramp	<= 1'b0; // complete the pulse.
						
						state <= COUNT_ROW;
						sel_row_period_cnt <= 0;
						
					end else begin
						state <= SELECT_ROW;
					end
				end
			end
			
			COUNT_ROW: begin // ramp down & count up based on clk_ramp==clk_in.
				//outputs
				clk_sel <= 1'b0;
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				ramp_swRst	<= 1'b1; // start the ramp down.
				en_count	<= 1'b1; // enable the counter.
				rst_shift <= 1'b1; // reset the multiplexer to prepare for READ.
				
				//internals
				
				//state
				count_row_period_cnt <= count_row_period_cnt + 1;
				if((count_row_period_cnt >= count_row_period)) begin
					enRow_init	<= 1'b0; // complete the enrow_init pulse.
					en_count	<= 1'b0; // disable the counter.
					rst_shift <= 1'b0; // complete the pulse.
					ramp_swRst	<= 1'b0; // reset the V_int for the next ramp down. Low active PMOS switch.
					en_shift <= 1'b1; // enable the shift register (multiplexer).
					
					state <= READ_ROW;
					count_row_period_cnt <= 0;
				end else begin
					state <= COUNT_ROW;
				end
			end
			
			READ_ROW: begin // multiplexer operation to deliver read bits to the pads. 10 by 10.
				//outputs
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				ramp_swRst	<= 1'b0; // reset the V_int for the next ramp down. Low active PMOS switch.
				en_count	<= 1'b0; // disable the counter.
				req_fifowr <= 1'b0;
				
				clk_shift <= 1'b1;
				if (read_row_period_cnt >= 4'd10) begin // when we reach 10, stop the row reading.
					en_shift <= 1'b0; // disable the shift register (multiplexer) if the period is over.
					clk_shift <= 1'b0;
					
					data_wait_cnt <= 4'b0;
					read_row_period_cnt <= 4'b0;
					
					state <= RESET_ROW;
				
				end else begin	
					if (data_wait_cnt >= data_wait_cycles) begin	
						if(rowCount == 0) begin
							dout <= {read_row_period_cnt, 1'b1, 1'b0, DIN}; // add an indicator for the start of each frame.
						end else begin
							dout <= {read_row_period_cnt, 2'b00, DIN};
						end
						
						// add an extra delay before FIFO, to settle dout. Added delay: one clock cycle only.
						fifo_delay_cnt <= fifo_delay_cnt + 1;
						if(fifo_delay_cnt >= 1) begin
							req_fifowr <= 1'b1; 				//send it bro!!
							read_row_period_cnt <= read_row_period_cnt + 1; // shiftedPixelCount. Counting the number of pixels shifted in a given row.
							
							clk_shift <= 1'b0;
							data_wait_cnt <= 4'b0;
							fifo_delay_cnt <= 2'b0;
							state <= READ_ROW;
						end else begin
							state <= READ_ROW;
						end
					end else begin
						data_wait_cnt <= data_wait_cnt + 1'b1;
						state <= READ_ROW;
					end
				end
			end
			
			RESET_ROW: begin // 
				//outputs
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				clk_rst <= 1'b1; // reset the selected row.
				rst_ramp <= 1'b1; // reset the counter outputs.
				
				//internals
				
				//state
				reset_row_period_cnt <= reset_row_period_cnt + 1;
				if((reset_row_period_cnt >= reset_row_period)) begin
					clk_rst <= 1'b0; // complete the clk_rst pulse.
					rst_ramp <= 1'b0; // pulse completed.
					
					state <= COUNT_RESET_ROW;
					reset_row_period_cnt <= 0;
				end else begin
					state <= RESET_ROW;
				end
			end
			
			COUNT_RESET_ROW: begin // ramp down & count up based on clk_ramp==clk_in.
				//outputs
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				ramp_swRst	<= 1'b1; // start the ramp down.
				en_count	<= 1'b1; // enable the counter.
				rst_shift <= 1'b1; // reset the multiplexer to prepare for READ_RESET.
				
				//internals
				
				//state
				count_reset_row_period_cnt <= count_reset_row_period_cnt + 1;
				if((count_reset_row_period_cnt >= count_reset_row_period)) begin
					en_count	<= 1'b0; // disable the counter.
					rst_shift <= 1'b0; // complete the pulse.
					ramp_swRst	<= 1'b0; // reset the V_int for the next ramp down. Low active PMOS switch.
					en_shift <= 1'b1; // enable the shift register (multiplexer).
					
					state <= READ_RESET_ROW;
					count_reset_row_period_cnt <= 0;
				end else begin
					state <= COUNT_RESET_ROW;
				end
			end
			
			READ_RESET_ROW: begin // multiplexer operation to deliver read bits to the pads. 10 by 10.
				//outputs
				enRow_rst	<= 1'b1; // needs to stay high during readout. Low active.
				ramp_swRst	<= 1'b0; // reset the V_int for the next ramp down. Low active PMOS switch.
				en_count	<= 1'b0; // disable the counter.
				req_fifowr <= 1'b0;
				
				clk_shift <= 1'b1;
				if (read_reset_row_period_cnt >= 4'd10) begin // when we reach 10, stop the row reading.
					en_shift <= 1'b0; // disable the shift register (multiplexer) if the period is over.
					clk_shift <= 1'b0;
					
					data_wait_cnt <= 4'b0;
					read_reset_row_period_cnt <= 4'b0;
					rowCount <= rowCount + 1; // increase the RowCount
					
					state <= SELECT_ROW;
				
				end else begin	
					if (data_wait_cnt >= data_wait_cycles) begin	
						if(rowCount == 0) begin
							dout <= {read_reset_row_period_cnt, 1'b1, 1'b0, DIN}; // add an indicator for the start of each frame.
						end else begin
							dout <= {read_reset_row_period_cnt, 2'b00, DIN};
						end
						
						// add an extra delay before FIFO, to settle dout. Added delay: one clock cycle only.
						fifo_delay_cnt <= fifo_delay_cnt + 1;
						if(fifo_delay_cnt >= 1) begin
							req_fifowr <= 1'b1; 				//send it bro!!
							read_reset_row_period_cnt <= read_reset_row_period_cnt + 1; // shiftedPixelCount. Counting the number of pixels shifted in a given row.
							
							clk_shift <= 1'b0;
							data_wait_cnt <= 4'b0;
							fifo_delay_cnt <= 2'b0;
							state <= READ_RESET_ROW;
						end else begin
							state <= READ_RESET_ROW;
						end
					end else begin
						data_wait_cnt <= data_wait_cnt + 1'b1;
						state <= READ_RESET_ROW;
					end
				end
			end
			
			default: begin //same behaviour as reset clause
				//outputs
				glob_tx	<= 1'b0;
				glob_grst	<= 1'b0;
				clk_sel	<= 1'b0;
				clk_rst	<= 1'b0;
				enRow_rst	<= 1'b0;
				enRow_init	<= 1'b0;
				ramp_swRst	<= 1'b0;
				//clk_ramp	<= 1'b0;
				rst_ramp	<= 1'b0;
				en_count	<= 1'b0;
				clk_shift	<= 1'b0;
				rst_shift	<= 1'b0;
				en_shift	<= 1'b0;
				
				READ_EN <= 1'b0;
				MEM_CLEAR <= 1'b0;
				//internals
				rowCount	<= 10'b0;
				frameCount <= 10'b0;
				//state
				state <= IDLE;
			end
			
		endcase
	end
end

endmodule
