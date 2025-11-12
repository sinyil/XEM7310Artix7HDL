#%%First Comments
##This file is written to extract data from the board Sinan designed with XEM7310/Artix-7.
##-Find, enable and flash the OK
##-Enable the FSM
##-Draw out the data single time, calculate the transfer speed
#
##The main routine does the following
##-Draw out data constantly via opal kelly pipe interface
##-De-interleave the data
##
# ep00wire - STD_LOGIC_VECTOR(31 downto 0)
# --hex0: resets
#			--bit 0 : reset
#			--bit 1 : unused --used to be drp_reset
#			--bit 2 : ram reset
#			--bit 3 : fsm_reset
#	--hex1: datapath enable
#			--bit 4 : unused --used to be RAM write_enable
#			--bit 5 : unused --used to be RAM read_enable
#			--bit 6 : unused --used to be calib_int
#			--bit 7 : unused --used to be valid_int
#	--hex2: FSM settings
#			--bit 8 : fsm_enable
#			--bit 9 : unused --used to be fsm_delay
#			--bit 10 : unused
#			--bit 11 : unused --clk_gen_rst
#	--hex3: PLL settings
#			--bit 12 : unused --used to be SPAD_ON_clk_sel mux
#			--bit 13 : unused --used to be pll_clkinsel --separate bitfiles for int and ext clocks instead.
#			--bit 14 : unused -- used to be ce_spadout
#			--bit 15 : unused -- used to be ce_laserclk
#	--hex4: IC settings
#			--bit 16 : unused -- used to be RSTB
#			--bit 17 : unused -- used to be SRESET
#			--bit 18 : unused
#			--bit 19 : unused
#
# ep01wire - STD_LOGIC_VECTOR(31 downto 0)
#			--bits 0:9 : fsm_global_rst_period
#
# ep02wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:31 : fsm_integration_period
#
# ep03wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bit 0:3 : fsm_data_wait_cycles
#
# ep04wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 28 downto 0 : user_memory_limit
#
# ep10wire - STD_LOGIC_VECTOR(31 downto 0)
#           --all bits : fsm_numRows
#
# ep11wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:9 : fsm_transfer_period
#
# ep12wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:9 : fsm_sel_row_period
#
# ep13wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:9 : fsm_count_row_period
#
# ep14wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:9 : fsm_reset_row_period
#
# ep15wire - STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:9 : fsm_count_reset_row_period
#
# ep20wire: STD_LOGIC_VECTOR(31 downto 0)
#           --LSB: fifo1_wr_data_count
#			--MSB: fifo1_rd_data_count
#
# ep21wire: STD_LOGIC_VECTOR(31 downto 0)
#           --LSB: fifo2_wr_data_count
#			--MSB: fifo2_rd_data_count
#			
# ep22wire: STD_LOGIC_VECTOR(31 downto 0)
#           --LSB: output_fifo_wr_data_count
#			--MSB: output_fifo_rd_data_count
#
# ep23wire: STD_LOGIC_VECTOR(31 downto 0)
#           --bit 0 : write_phase_complete
#			--bit 1 : read_phase_complete
#			--bits 2:4 : mem_mode
#			--bit 5 : memoryFull
#			--bit 6 : po0_ep_read
#			--bit 7 : pipe_out_ready
#			--bits 8:27 : unused ----------
#			--bits 28:31 : mem_state
#
# ep24wire: STD_LOGIC_VECTOR(31 downto 0)
#           --bit 0 : clk_locked --used to be pll_locked
#			--bit 1 : init_calib_complete --used to be c3_pll_lock
#			--bit 2 : ce_clk_generated
#			--bits 3:31 : output_fifo_wr_data_count
#
# ep25wire: STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:28 : output_fifo_rd_data_count
#
# ep26wire: STD_LOGIC_VECTOR(31 downto 0)
#           --bits 0:29 : memoryCount
#
# TRIGGEROUT
# ep60TrigOut: STD_LOGIC_VECTOR(31 downto 0)
#           --pipe_out_ready
#%%Run Setup
import sys
import time
from datetime import datetime
import ok
import argparse
import numpy as np
import plot_script
import pyvisa
import subprocess

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
 
 voltageRange = [x/100.0 for x in range(292, 293, 1)]
 SNRs = []
 DCRs = []
 SBRs = []
 reset_maxs = []
 reset_mins = []
 reset_noises = []
 mean_resets = []
 DRs = []
 for v_s in voltageRange:
    # to set the voltage levels for both channels on the Keysight B2962A power source.
    rm = pyvisa.ResourceManager()
    voltageSource = rm.open_resource('USB0::0x0957::0x9018::MY52350820::INSTR') # to reset the instrument: voltageSource.write('*RST')
    voltageSource.write('source1:voltage 2.5')
    voltageSource.write('sense1:current:protection 0.01')
    voltageSource.write('output1 1') # channel on/off
    voltageSource.write('source2:voltage ' + str(v_s))
    voltageSource.write('sense2:current:protection 0.01')
    voltageSource.write('output2 1') # channel on/off
    time.sleep(0.1)
    
    folder = 'C:/Users/opticsroom/Documents/sinan'
    project_dir = folder + '/20250922_opalKelly7310_python'
    
    # --------------------Bitfiles--------------------
    bitfile = folder + '/bitfiles/pdShank_v7310.bit'; clk_in = 200e6;
	
    # --------------------Bitfiles--------------------
    
    chipType = 0  # == 1 if only photodiodes. == 0 if there are uLED pads
    pixelArrayStr = '100x10'  # second term, number of pixels per row is always 10 for pdShank2022 bitfiles above. #pixelArray = bitfile.split('_')[-1].split('.')[0]
    pixelArray = [eval(i) for i in pixelArrayStr.split('x')]
    numRows = pixelArray[0]
    
    # -------chipType 0: For 4000-pixel shank with uLED pads. 1 Full Frame = 62.5 blocks (BLOCK_SIZE = 256 bytes). " x totalNumFrames x (pixelArrayStr/8000)" needs to be integer.
    # largest pixelArray = [400, 10]
    # -------chipType 1: For 8000-pixel shank with only PDs. 1 Full Frame = 125 blocks (BLOCK_SIZE = 256 bytes). " x totalNumFrames x (pixelArrayStr/8000)" needs to be integer.
    # largest pixelArray = [800, 10]
    
    clock_frequency = clk_in  # 200 * (10**6)  # [Hz] .......
    clock_period = 1 / clock_frequency  # [sec]
    clock_period_ns = clock_period * (10 ** 9)  # 5 [ns]
    
    clock_duty = 50
    clock_phase = 0
    
    test_name = 'v4_intlog_gtx_rem_' + "{:.2f}".format(v_s) + '_xem7310_test'
    
    # These periods can cause the VHDL code to get stuck/unsynced in one of the write states if it coincides with the refresh rate of the DDR3.
    # Just slightly tweak the integration +-1 to make it work.
    global_rst_numCycle = 2**1-1 # Max = 1023 ......... [* clock_period_ns] ns
    integration_numCycle = 2**10-1  # Max = 1023 intlogMax = 2e32-1 ......... [* clock_period_ns] ns
    transfer_numCycle = 10  # Max = 1023 ............ [* clock_period_ns] ns
    sel_row_numCycle = 3  # Max = 1023 ......... [* clock_period_ns] ns
    count_row_numCycle = 1023  # Max = 1023 ......... [* clock_period_ns] ns
    reset_row_numCycle = 10  # Max = 1023 ............ [* clock_period_ns] ns○
    count_reset_row_numCycle = 1023  # Max = 1023 ............ [* clock_period_ns] ns
    
    data_wait_cycles = 7  # Max = 15, usually 7
    
    fps_oneFrame = 1/(clock_period*(2 + global_rst_numCycle + integration_numCycle + transfer_numCycle + pixelArray[0]*(sel_row_numCycle + count_row_numCycle + pixelArray[1]*(data_wait_cycles+1) + reset_row_numCycle + count_reset_row_numCycle + pixelArray[1]*(data_wait_cycles+1))))
    
    numFrames = 260000-10  # default 4 (Max: 262000 ~ 2**18 = 262144 if pixelArray: '100x10')
    ignoreFrames = 10  # default 4.. at least 4 for pdshank2022
    numChunks = 1  # default 1
    
    total_duration = numFrames/fps_oneFrame
    downsamplingRatio = 1
    fps = fps_oneFrame/downsamplingRatio
    
    numPixels = pixelArray[0] * pixelArray[1]
    frame_numBits = (16 * pixelArray[1] + 16 * pixelArray[1]) * pixelArray[0] # includes the delta reset as well
    
    arg_get = 1; arg_read = 0; arg_plot = 0
    
    arg_output_file = project_dir + '/rawdata/' + test_name
    
    arg_ep00 = int('0b' + '0000' + '0000' + '0001' + '0000' + '0000', 2)  # fsm_enable & 0 & 0 & RAM_read_en & RAM_write_en
    
    arg_ep01 = int('0b' + '{0:010b}'.format(global_rst_numCycle), 2)
    arg_ep02 = int('0b' + '{0:032b}'.format(integration_numCycle), 2)
    arg_ep11 = int('0b' + '{0:010b}'.format(transfer_numCycle), 2)
    arg_ep12 = int('0b' + '{0:010b}'.format(sel_row_numCycle), 2)
    arg_ep13 = int('0b' + '{0:010b}'.format(count_row_numCycle), 2)
    arg_ep14 = int('0b' + '{0:010b}'.format(reset_row_numCycle), 2)
    arg_ep15 = int('0b' + '{0:010b}'.format(count_reset_row_numCycle), 2)
    
    arg_ep03 = int('0b' + '{0:04b}'.format(data_wait_cycles), 2)
    arg_ep10 = int('0b' + '{0:010b}'.format(numRows), 2)
    
    totalNumFrames = numFrames + ignoreFrames
    numBytesTransferringPerFrame = frame_numBits / 8
    numBytesTransferring = numBytesTransferringPerFrame * totalNumFrames  # frame_numBits=16000*16 for 800-pixel shank, frame_numBits=8000*16 for 400-pixel shank
    user_memory_limit_addresswise = numBytesTransferring / 4 # user_memory_limit is crosschecked with the write address to stop the FSM and memory write process. Each address holds 4 bytes. (32 bits) The maximum address: 10**28-1 = 268435456-1.
    dataSizePerChunk_in_MB = numBytesTransferring / 1024 / 1024
    
    numPixReadings = frame_numBits / 16
    
    BlockSize = 256  # PIPEOUT block size. (in bytes) Important to be the same as in the VHDL code otherwise data could be different.
    arg_ep04 = int('0b' + '{0:030b}'.format(int(user_memory_limit_addresswise+BlockSize)), 2) # + BlockSize is there to be safe in terms of BLOCK readout.
    
    arg_reset = 1
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--reset', nargs='?', default=arg_reset, type=int, help='Full reset y/n')
    parser.add_argument('-rstc', '--resetcode', nargs='?', default=15, type=int, help='Reset code')  # 15 to reset fsm as well.
    parser.add_argument('-ep00', '--ep00wire', nargs='?', default=arg_ep00, type=int, help='ep00wire')  # to start fsm and enable RAM after reset.
    parser.add_argument('-ep03', '--ep03wire', nargs='?', default=arg_ep03, type=int, help='ep03wire')  # fsm delay length in clock cycles. read_row and read_reset_row use this.
    
    parser.add_argument('-ep04', '--ep04wire', nargs='?', default=arg_ep04, type=int, help='ep04wire')  # requested data size in number of addresses (Bytes/4)
    
    parser.add_argument('-ep10', '--ep10wire', nargs='?', default=arg_ep10, type=int, help='ep10wire')  # numRows
    
    parser.add_argument('-ep01', '--ep01wire', nargs='?', default=arg_ep01, type=int, help='ep01wire')  # periods start
    parser.add_argument('-ep02', '--ep02wire', nargs='?', default=arg_ep02, type=int, help='ep02wire')  
    parser.add_argument('-ep11', '--ep11wire', nargs='?', default=arg_ep11, type=int, help='ep11wire')  
    parser.add_argument('-ep12', '--ep12wire', nargs='?', default=arg_ep12, type=int, help='ep12wire')  
    parser.add_argument('-ep13', '--ep13wire', nargs='?', default=arg_ep13, type=int, help='ep13wire')  
    parser.add_argument('-ep14', '--ep14wire', nargs='?', default=arg_ep14, type=int, help='ep14wire')  
    parser.add_argument('-ep15', '--ep15wire', nargs='?', default=arg_ep15, type=int, help='ep15wire')  # periods end
    
    parser.add_argument('-td', '--total_duration', nargs='?', default=total_duration, type=int, help='Total duration')
    parser.add_argument('-nf', '--numFrames', nargs='?', default=numFrames, type=int, help='Number of frames')
    parser.add_argument('-if', '--ignoreFrames', nargs='?', default=ignoreFrames, type=int, help='Number of ignored frames')
    parser.add_argument('-nc', '--numChunks', nargs='?', default=numChunks, type=int, help='Number of snaps')  # always 1 for now. But reliable now with 156.25 MHz clock.
    parser.add_argument('-ff', '--flashfile', nargs='?', default=bitfile, type=str, help='Flash file')
    parser.add_argument('-pa', '--pixelArray', nargs='?', default=pixelArrayStr, type=str, help='Pixel Array')  # [800, 10] or [400, 10]
    parser.add_argument('-o', '--outputfile', nargs='?', default=arg_output_file, type=str, help='Output file')
    parser.add_argument('-get', '--getdata', nargs='?', default=arg_get, type=int, help='Get data')
    parser.add_argument('-read', '--readdata', nargs='?', default=arg_read, type=int, help='Read data')
    parser.add_argument('-plot', '--plotdata', nargs='?', default=arg_plot, type=int, help='Plot data')
    args = parser.parse_args()
    
    error_list = []
    # COLLECT DATA
    if args.getdata == 1:

        # to flash the OK
        device = ok.okCFrontPanel()  # this class is used to interface with fpga. ok.okCFrontPanelDevices()  # this class is only used for device query. --> deviceQuery.GetSerial(0)
        device.OpenBySerial('2205000Z10') #2205000Z10 #2436001CXW

        print(f"Is open? {device.IsOpen()}")

        flerr = device.ConfigureFPGA(args.flashfile)
        if flerr == 0:
            print('Opal Kelly Flash Successful.')
        else:
            print('Opal Kelly Flash Error: error code ', flerr)
            sys.exit('Opal Kelly Flash is not successful.')

        if device.IsFrontPanelEnabled():  # IsFrontPanelEnabled returns true if FrontPanel is detected.
            print('FrontPanel host interface enabled.')
            pass
        else:
            sys.stderr.write('FrontPanel host interface not detected.')

#%% EXECUTION ----------- ----------- ----------- ----------- ----------- ----------- ----------- ----------- -----------
        
        # FSM and MEMORY Setup -------WIREINS
        device.SetWireInValue(0x01, args.ep01wire)  # periods start
        device.SetWireInValue(0x02, args.ep02wire)  
        device.SetWireInValue(0x11, args.ep11wire)  
        device.SetWireInValue(0x12, args.ep12wire)  
        device.SetWireInValue(0x13, args.ep13wire)  
        device.SetWireInValue(0x14, args.ep14wire)  
        device.SetWireInValue(0x15, args.ep15wire)  # periods end
          
        device.SetWireInValue(0x03, args.ep03wire)  # waiting time for settling is given here.
          
        device.SetWireInValue(0x10, args.ep10wire) # numRows
        device.SetWireInValue(0x04, args.ep04wire) # user_memory_limit
        device.UpdateWireIns()
        
        # RESET EVERYTHING
        if args.reset == 1:
            print('     Fully resetting device')
            error_list.append(device.SetWireInValue(0x00, int(args.resetcode)))  # 15 turns on first four elements of ep00wire (resets).
            device.UpdateWireIns()
            time.sleep(0.5)
            # set everything to reset
            device.SetWireInValue(0x00, int('0b' + '0000' + '0000' + '0000' + '0000' + '0000', 2))
            device.UpdateWireIns()
            
        # START THE FSM. AFTER the clocks have been settled. VHDL code internally checks this.
        device.SetWireInValue(0x00, int(args.ep00wire))  # fsm_enable & 0 & 0 & (RAM_read_en & RAM_write_en)
        device.UpdateWireIns()
        
#%% PIPEOUT
        # --- PIPEOUT--- to collect the data from OK... BLOCK_SIZE is defined above.
        device.UpdateWireOuts()
        MemoryCount = device.GetWireOutValue(0x26)
        print('     Current memory count is: %d ' % MemoryCount)

        timestamp = np.zeros(args.numChunks+1)  # First element is the ts0. Others are increments from that point.
        ts0 = time.time()  # timestamp_0

        for c in range(0, args.numChunks):

            device.UpdateWireOuts()
            MemoryCount = device.GetWireOutValue(0x26)

            # ------------Nice to have the TransferSize (data_out) as multiple of 256 bytes that is BLOCK_SIZE.
            # ------------16*10 read_row, 16*10 read_reset_row for numRows. [Delta Sampling: read_row first, then read_reset_row]
            # ------------First a few frames are always contaminated for every chunk. Get more than requested and get rid of extra ones during data processing.

            numBlocksTransferring = numBytesTransferring / BlockSize
            numBlocksTransferringPerFrame = numBlocksTransferring / totalNumFrames
            data_out = bytearray(int(numBlocksTransferring) * BlockSize)  # Bytes.
            TransferSize = len(data_out)  # bytes. multiple of 256 (BLOCK_SIZE) bytes.

            while MemoryCount < TransferSize: # Justification for division by 8: we should start the pipeOut earlier. Otherwise Error -2. Timeout.
                device.UpdateWireOuts()
                MemoryCount = device.GetWireOutValue(0x26)
                
               	device.UpdateWireOuts()
                print('20 '+'{:032b}'.format(device.GetWireOutValue(0x20)))
                print('21 '+'{:032b}'.format(device.GetWireOutValue(0x21)))
               	print('22 '+'{:032b}'.format(device.GetWireOutValue(0x22)))
               	print('23 '+'{:032b}'.format(device.GetWireOutValue(0x23)))
                print('24 '+'{:032b}'.format(device.GetWireOutValue(0x24)))
                print('25 '+'{:032b}'.format(device.GetWireOutValue(0x25)))
                
                time.sleep(0.01)  # It takes 1 second to fill up the memory to 64MB, so it is useless to check 100000x per second
                print(f'Status 2: memory count = {MemoryCount} whereas transfer size = {TransferSize} Bytes = {TransferSize/1024} KB')
                
            if MemoryCount >= TransferSize: # Justification for division by 8: we should start the pipeOut earlier. Otherwise Error -2. Timeout.
                code = device.ReadFromBlockPipeOut(0xa0, BlockSize, data_out)  # if no error, number of bytes; else error code.

                timestamp[0] = ts0  # First element is ts0.
                timestamp[c+1] = time.time() - ts0  # Others are increments from ts0.
                print(f'     Timestamp for chunk {c} is: {timestamp[c+1]} sec ==> {datetime.fromtimestamp(ts0+timestamp[c+1])} ')

                if code == TransferSize:
                    print(f'     TransferSize matched. Data Retrieved: {float(int(code)) / 1024} kB ({totalNumFrames} total frames in this chunk, including the contaminated ones)')
                    with open(args.outputfile + '_chunk_' + str(c), 'wb') as f:
                        f.write(data_out)
                    np.savetxt(args.outputfile + '_timestamp_' + str(c) + '.txt', np.array([datetime.fromtimestamp(ts0).strftime("%Y-%m-%d %H:%M:%S.%f"), datetime.fromtimestamp(ts0+timestamp[c+1]).strftime("%Y-%m-%d %H:%M:%S.%f")]), fmt="%s")
                    #with open(args.outputfile + '_timestamp2.txt', 'wb') as f:  # to directly write timestamp data with python file manager in bytes
                    #    f.write(timestamp.tobytes())  # works even without np.tobytes() function. This gets rid of warnings though.
                    
                    # --------------->>>>>>>> Transfer it to the server. VPN needed.
                    return_code = subprocess.run('"C:\\Program Files (x86)\\WinSCP\\WinSCP.com" /command "option batch abort" "option confirm off" "open sftp://syilmaz:Kestel!110@bioeebeanie.ee.columbia.edu/" "put ' + args.outputfile.replace('/', '\\') + '_chunk_' + str(c) + ' /users/syilmaz/Python/20250922_opalKelly7310_python/rawdata/"  "exit"')
                    return_code = subprocess.run('"C:\\Program Files (x86)\\WinSCP\\WinSCP.com" /command "option batch abort" "option confirm off" "open sftp://syilmaz:Kestel!110@bioeebeanie.ee.columbia.edu/" "put ' + args.outputfile.replace('/', '\\') + '_timestamp_' + str(c) + '.txt' + ' /users/syilmaz/Python/20250922_opalKelly7310_python/rawdata/"  "exit"')
                else:
                    print(f'     TransferSize does not match. TransferSize: {TransferSize} & Code: {code}')
                    print('     Did not write file!')
                    sys.stderr.write("     Data was not retrieved correctly. Error code returned is %d" % code)
                    sys.exit('      Data is not even saved. PipeOut issue.')

                # reset RAM
                print('Resetting RAM:')
                device.UpdateWireOuts()
                MemoryCount = device.GetWireOutValue(0x26)
                print('     Memory count before the reset: %d ' % MemoryCount)

                device.SetWireInValue(0x00, int(args.resetcode))  # resetting fsm as well. Should we?
                device.UpdateWireIns()
                time.sleep(0.1)
                device.SetWireInValue(0x00, int('0b'+'0000'+'0000'+'0000'+'0000'+'0000', 2)) # FSM is not enabled here, following the reset after data acquisition.
                device.UpdateWireIns()

                device.UpdateWireOuts()
                MemoryCount = device.GetWireOutValue(0x26)
                print('     Memory count after the reset: %d ' % MemoryCount)
                print(f'End of chunk {c}')
    
    device.Close()            
    
    # READ ALL THE CHUNKS
    if args.readdata == 1:

        fullData_binary_chunks = []
        for ch in range(args.numChunks):
            with open(args.outputfile + '_chunk_' + str(ch), 'rb') as fid:
                A = np.fromfile(fid, np.uint16)  # .astype(np.float64)

            B = ['{0:016b}'.format(A[i]) for i, data in enumerate(A)]  # convert all to binary with 16 bits. B is the full data including numFrames and ignoreFrames.

            pixCountAndFrameFlagBits = [fsmdata16b[:6] for fsmdata16b in B]  # first 6 bits are dedicated for [4b-pixel count in a row] [1b-frameBeginFlag] [0]

            frameBeginIndices = [i for i, x in enumerate(pixCountAndFrameFlagBits) if x == '000010']  # flag is used to detect first rows of each frame.
            chunk_diff = np.diff(frameBeginIndices)  # diff list should go like [10, 15990, 10, 15990 ...] since we have read_reset_row after read_row for each row.

#%%Intact Frame Detection
            # to read the intact frames, beginning from the end. only count complete frames. when you reach numFrames, stop.
            B_extracted_numFrames = []; count=0;
            for i, num in enumerate(reversed(chunk_diff)):
                if chunk_diff[-1-i] == numPixReadings-pixelArray[1] and chunk_diff[-2-i] == pixelArray[1]:
                    B_extracted_numFrames.append(B[frameBeginIndices[-3-i]:frameBeginIndices[-1-i]])
                    count = count + 1
                    if count == args.numFrames:
                        break
                else:
                    continue
            
            B_extracted_numFrames.reverse() # Reversed. Since we're reading from the last frame to first. We got to reverse it back.
            B_extracted_numFrames = [pix for frame in B_extracted_numFrames for pix in frame] #flatten it again
            
            numIntactFramesReadPerChunk = len(B_extracted_numFrames) / numPixReadings
            if args.numFrames == numIntactFramesReadPerChunk:
                print(f'        Chunk_{ch}: Number of intact frames read ({numIntactFramesReadPerChunk}) matched with number of frames requested ({args.numFrames})')
                fullData_binary_chunks.append(B_extracted_numFrames)
            else:
                print(f'        Chunk_{ch}: Number of intact frames read ({numIntactFramesReadPerChunk}) did not match with number of frames requested ({args.numFrames})')
                sys.stderr.write('      Not included in the fullData. Rerun the measurement script if necessary.')

#Timestamp Read -----to read the timestamps file.
        #incremented_timestamps = np.loadtxt(args.outputfile + '_timestamp.txt')
        #for chu in range(args.numChunks):
        #    print(f'Timestamp for chunk {chu}: {datetime.fromtimestamp(incremented_timestamps[0]+incremented_timestamps[c+1])}')

#%%     -----DATA PROCESSING----- fullData_binary
        # to use delta sampling, go over the 'fullData_binary' and get measured data.
        fullData_binary = np.array(fullData_binary_chunks).flatten()  # [item for sublist in fullData_binary_chunks for item in sublist]
        numIntactFramesReadAllChunks = int(len(fullData_binary) / numPixReadings)
        # fullData_deltaSampled
        #--------------- No flipping the binary data. We'll just subtract pixReadings from resetReadings to fight the mispolarity of the comparator.
        fullData = np.reshape([int(p[-10:], 2) for p in fullData_binary], (numIntactFramesReadAllChunks, pixelArray[0], pixelArray[1]*2), order = 'C')
        
        fullData_resetReadings = fullData[:, :, pixelArray[1]:]
        fullData_pixReadings = fullData[:, :, :pixelArray[1]]
        
        #--------------- Subtracting pixReadings from resetReadings!!! because of the mispolarity of the comparator.
        fullData_deltaSampled = np.subtract(fullData_resetReadings, fullData_pixReadings)
        
#%%--------------------------# with_uLED (chipType = 0) Corners are more responsive than centers.
        # if chipType == 0:
        #     fullData_deltaSampled[:,10:-1:10,0] = 0.519 * fullData_deltaSampled[:,10:-1:10,0]; fullData_deltaSampled[:,9:-1:10,0] = 0.519 * fullData_deltaSampled[:,9:-1:10,0]
        #     fullData_deltaSampled[:,10:-1:10,1] = 0.634 * fullData_deltaSampled[:,10:-1:10,1]; fullData_deltaSampled[:,9:-1:10,1] = 0.634 * fullData_deltaSampled[:,9:-1:10,1]
        #     fullData_deltaSampled[:,10:-1:10,2] = 0.682 * fullData_deltaSampled[:,10:-1:10,2]; fullData_deltaSampled[:,9:-1:10,2] = 0.682 * fullData_deltaSampled[:,9:-1:10,2]
        #     fullData_deltaSampled[:,10:-1:10,3] = 0.700 * fullData_deltaSampled[:,10:-1:10,3]; fullData_deltaSampled[:,9:-1:10,3] = 0.700 * fullData_deltaSampled[:,9:-1:10,3]
        #     fullData_deltaSampled[:,10:-1:10,4] = 0.701 * fullData_deltaSampled[:,10:-1:10,4]; fullData_deltaSampled[:,9:-1:10,4] = 0.701 * fullData_deltaSampled[:,9:-1:10,4]
        #     fullData_deltaSampled[:,10:-1:10,5] = 0.685 * fullData_deltaSampled[:,10:-1:10,5]; fullData_deltaSampled[:,9:-1:10,5] = 0.685 * fullData_deltaSampled[:,9:-1:10,5]
        #     fullData_deltaSampled[:,10:-1:10,6] = 0.672 * fullData_deltaSampled[:,10:-1:10,6]; fullData_deltaSampled[:,9:-1:10,6] = 0.672 * fullData_deltaSampled[:,9:-1:10,6]
        #     fullData_deltaSampled[:,10:-1:10,7] = 0.620 * fullData_deltaSampled[:,10:-1:10,7]; fullData_deltaSampled[:,9:-1:10,7] = 0.620 * fullData_deltaSampled[:,9:-1:10,7]
        #     fullData_deltaSampled[:,10:-1:10,8] = 0.517 * fullData_deltaSampled[:,10:-1:10,8]; fullData_deltaSampled[:,9:-1:10,8] = 0.517 * fullData_deltaSampled[:,9:-1:10,8]
        #     fullData_deltaSampled[:,10:-1:10,9] = 0.606 * fullData_deltaSampled[:,10:-1:10,9]; fullData_deltaSampled[:,9:-1:10,9] = 0.606 * fullData_deltaSampled[:,9:-1:10,9]
            
#--------------------------# Last column is ~x2 more sensitive. PixelResponseNonUniformity. (PRNU) 
        #fullData_deltaSampled[:,:,9] = 0.354* fullData_deltaSampled[:,:,9]
        fullData_deltaSampled_sum = np.sum(fullData_deltaSampled, axis=0)
        
        #fullData_deltaSampled_sum[:,9] = fullData_deltaSampled_sum[:,9]
        #fullData_deltaSampled_sum[:,9] = 50 * np.exp(0.003*fullData_deltaSampled_sum[:,9]/numFrames)*numFrames  # v4_ intlog 0.45 when numFrames=123. 0.5 when numFrames=3
        #fullData_deltaSampled_sum[:,9] = numFrames*20*np.emath.sqrt(0.4*fullData_deltaSampled_sum[:,9]/numFrames)
        #fullData_deltaSampled_sum[:,9] = 0.354* fullData_deltaSampled_sum[:,9]
        
        # v4_intlog_gtx_rem ---> v_s == 2.92:
        #fullData_deltaSampled_sum[:,9] = numFrames*(0.479*(fullData_deltaSampled_sum[:,9]/numFrames)-1.118)  # v4_intlong_gtx_rem with 2.92V: [FirstRow = 0.479*LastRow-1.118]
        
        # v4_intlog_gtx_rem ---> v_s == 2.85:
        #fullData_deltaSampled_sum[:,9] = numFrames*(344.302*np.log(0.003*fullData_deltaSampled_sum[:,9]/numFrames+0.977))

#%% PixReadings and ResetReadings saved and plotted separately as well.       
        #DCR = 97
        #DCRmask = fullData_deltaSampled_sum[:,9]/numFrames <= DCR
        #mx = np.ma.masked_array(fullData_deltaSampled_sum[:,9], mask=DCRmask)
        
        fullData_pixReadings_sum = np.sum(fullData_pixReadings, axis=0)
        fullData_resetReadings_sum = np.sum(fullData_resetReadings, axis=0)
        
        # to extend the data if the chipType == 0
        ps = plot_script.plot_pdShank()
        all_extended_data = ps.extend_all_data(chipType, pixelArray, numIntactFramesReadAllChunks, fullData_resetReadings_sum, fullData_pixReadings_sum, fullData_deltaSampled_sum, fullData_resetReadings, fullData_pixReadings, fullData_deltaSampled, numPixels)
        # to save it in .npz file.
        np.savez(test_name, all_extended_data=all_extended_data, fps_oneFrame=fps_oneFrame, fps=fps, vStart=v_s, chipType=chipType, numFrames=numFrames, downsamplingRatio=downsamplingRatio, total_duration=total_duration, pixelArray=pixelArray, args=args)
        
    elif args.getdata == 1:
        args.plotdata = 0
        sys.exit('      Data is saved, but not read. No plots. Change arguments if necessary to read.')
    else:
        args.plotdata = 0
        sys.exit('      No data gotten. No data read. No plots. What was the point? Change arguments if necessary.')
        
    # to plot the data read
    if args.plotdata == 1:
        
        # 2D graph
        fig10 = ps.plot_2D(all_extended_data['fullData_pixReadings_sum_extended'], all_extended_data['fullData_resetReadings_sum_extended'], all_extended_data['fullData_deltaSampled_sum_extended'], all_extended_data['numPixels_extended'])
        
        # 3D graph
        #fig28 = ps.plot_3D(all_extended_data['fullData_deltaSampled_sum_extended'], all_extended_data['numPixels_extended'])
        
        # linear graphs
        fig17 = ps.plot_linear_graphs(numIntactFramesReadAllChunks, all_extended_data['numPixels_extended'], all_extended_data['fullData_pixReadings_extended'], all_extended_data['fullData_resetReadings_extended'], all_extended_data['fullData_deltaSampled_sum_extended'])
        
        fig10.savefig('2D_' + test_name + '.png', bbox_inches='tight')
        fig17.savefig('all_data_' + test_name + '.png', bbox_inches='tight')
        
        #max_location = np.unravel_index(all_extended_data.fullData_pixReadings_extended.argmax(), all_extended_data.fullData_pixReadings_extended.shape)
        
    print('End of the code.')  # Turn this into a function for continuous measurement during in vivo.
    
    #variance_Signal = ndimage.variance(all_extended_data['fullData_deltaSampled_sum_extended'][50:100,:8])
    #variance_BG = ndimage.variance(all_extended_data['fullData_deltaSampled_sum_extended'][50:60,:8])
    #SNRs.append(np.sqrt(variance_Signal/variance_BG - 1))
    
    #Signal = np.mean(all_extended_data['fullData_deltaSampled_sum_extended'][73:77,3:7])
    #BG = np.mean(np.abs(all_extended_data['fullData_deltaSampled_sum_extended'][50:60,:8]))
    #SBRs.append(Signal/BG)
    
    #DCRs.append(np.mean(np.abs(all_extended_data['fullData_deltaSampled_sum_extended'][-30:,:8])))
    
    #reset_maxs.append(np.max(all_extended_data['fullData_resetReadings_extended']))
    #reset_mins.append(np.min(all_extended_data['fullData_resetReadings_extended']))
    #mean_resets.append(np.mean(all_extended_data['fullData_resetReadings_extended']))
    #DRs = np.array(mean_resets)
    
 #reset_noises = np.array(reset_maxs) - np.array(reset_mins)
 #FoMs = abs(np.array(SBRs))*DRs/(abs(np.array(DCRs))*reset_noises)
 
 #np.save(test_name+'_DCRs', DCRs)
 #np.savez(test_name+'_all', SNRs=SNRs, SBRs=SBRs, DRs=DRs, reset_maxs=reset_maxs, reset_mins=reset_mins, reset_noises=reset_noises, mean_resets=mean_resets, FoMs=FoMs, voltageRange=voltageRange)
 
 