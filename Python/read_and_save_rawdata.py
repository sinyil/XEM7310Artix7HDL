import sys
#import time
#from datetime import datetime
#import ok
import argparse
import numpy as np
import plot_script
#import pyvisa
#import subprocess

v_s = 2.92
folder = '/users/syilmaz'
project_dir = folder + '/Python/20250922_opalKelly7310_python'

# --------------------Bitfiles--------------------
bitfile = folder + '/bitfiles/pdShank_v7310.bit'; clkdiv = 4; clk_in = 200e6; fvco = 800e6  # less DCR, less sensitive. @2.90V?

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

# These periods can cause the VHDL code to get stuck in one of the write states if it coincides with the refresh rate of the DDR3. Just slightly tweak the value +-1 to make it work.
global_rst_numCycle = 2**1-1 # Max = 1023 ......... [* clock_period_ns] ns
integration_numCycle = 2**10-1  # Max = 1023 intlogMax = 2e32-1 ......... [* clock_period_ns] ns
transfer_numCycle = 10  # Max = 1023 ............ [* clock_period_ns] ns
sel_row_numCycle = 3  # Max = 1023 ......... [* clock_period_ns] ns
count_row_numCycle = 1023  # Max = 1023 ......... [* clock_period_ns] ns
reset_row_numCycle = 10  # Max = 1023 ............ [* clock_period_ns] ns
count_reset_row_numCycle = 1023  # Max = 1023 ............ [* clock_period_ns] ns

data_wait_cycles = 7  # Max = 15, usually 7

fps_oneFrame = 1/(clock_period*(2 + global_rst_numCycle + integration_numCycle + transfer_numCycle + pixelArray[0]*(sel_row_numCycle + count_row_numCycle + pixelArray[1]*(data_wait_cycles+1) + reset_row_numCycle + count_reset_row_numCycle + pixelArray[1]*(data_wait_cycles+1))))

numFrames = 260000-10  # default 4 (Max: 262000 ~ 2**18 = 262144)
ignoreFrames = 10  # default 4.. at least 4 for pdshank2022
numChunks = 1  # default 1

total_duration = numFrames/fps_oneFrame
downsamplingRatio = 1
fps = fps_oneFrame/downsamplingRatio

arg_get = 0; arg_read = 1; arg_plot = 0

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

numPixels = pixelArray[0] * pixelArray[1]
frame_numBits = (16 * pixelArray[1] + 16 * pixelArray[1]) * pixelArray[0] # includes the delta reset as well

totalNumFrames = numFrames + ignoreFrames
numBytesTransferringPerFrame = frame_numBits / 8
numBytesTransferring = numBytesTransferringPerFrame * totalNumFrames  # frame_numBits=16000*16 for 800-pixel shank, frame_numBits=8000*16 for 400-pixel shank
user_memory_limit_addresswise = numBytesTransferring / 4 # user_memory_limit is crosschecked with the write address to stop the FSM and memory write process. Each address holds 4 bytes. (32 bits) The maximum address: 2**28-1 = 268435456-1
dataSizePerChunk_in_MB = numBytesTransferring / 1024 / 1024

numPixReadings = frame_numBits / 16

arg_ep04 = int('0b' + '{0:030b}'.format(int(user_memory_limit_addresswise+256)), 2) # + 256 is there to be safe in terms of BLOCK readout.

arg_reset = 1

parser = argparse.ArgumentParser()
parser.add_argument('-r', '--reset', nargs='?', default=arg_reset, type=int, help='Full reset y/n')
parser.add_argument('-rstc', '--resetcode', nargs='?', default=15, type=int, help='Reset code')  # 15 to reset fsm as well. if only RAM reset, 7.
parser.add_argument('-ep00', '--ep00wire', nargs='?', default=arg_ep00, type=int, help='ep00wire')  # to start fsm and enable RAM.

parser.add_argument('-ep01', '--ep01wire', nargs='?', default=arg_ep01, type=int, help='ep01wire')  # periods start
parser.add_argument('-ep02', '--ep02wire', nargs='?', default=arg_ep02, type=int, help='ep02wire')
parser.add_argument('-ep11', '--ep11wire', nargs='?', default=arg_ep11, type=int, help='ep11wire')
parser.add_argument('-ep12', '--ep12wire', nargs='?', default=arg_ep12, type=int, help='ep12wire')
parser.add_argument('-ep13', '--ep13wire', nargs='?', default=arg_ep13, type=int, help='ep13wire')
parser.add_argument('-ep14', '--ep14wire', nargs='?', default=arg_ep14, type=int, help='ep14wire')
parser.add_argument('-ep15', '--ep15wire', nargs='?', default=arg_ep15, type=int, help='ep15wire')  # periods end

parser.add_argument('-ep03', '--ep03wire', nargs='?', default=arg_ep03, type=int, help='ep03wire')  # fsm delay length in clock cycles. read_row and read_reset_row use this.
parser.add_argument('-ep04', '--ep04wire', nargs='?', default=arg_ep04, type=int, help='ep04wire')  # requested data size in number of addresses (Bytes/4)
#parser.add_argument('-clkdiv', '--clkdiv', nargs='?', default=arg_clkdiv, type=int, help='clkdivisions')  # ep04: PLL freq multiplier.
#parser.add_argument('-c0p', '--clkout0phase', nargs='?', default=arg_clkPhase, type=int, help='clkout0phase')  # ep05:
#parser.add_argument('-c0d', '--clkout0duty', nargs='?', default=arg_clkDuty, type=int, help='clkout0duty')  # ep06:
parser.add_argument('-ep10', '--ep10wire', nargs='?', default=arg_ep10, type=int, help='ep10wire')  # numRows
#parser.add_argument('-rpp', '--reprogpll', nargs='?', default=arg_rppll, type=int, help='reprogram pll')
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

        B_extracted_numFrames.reverse() # Reversed. Since we're reading from the last frame to first.
        B_extracted_numFrames = [pix for frame in B_extracted_numFrames for pix in frame] #flatten it again

        numIntactFramesReadPerChunk = len(B_extracted_numFrames) / numPixReadings
        if args.numFrames == numIntactFramesReadPerChunk:
            print(f'        Chunk_{ch}: Number of intact frames read ({numIntactFramesReadPerChunk}) matched with number of frames requested ({args.numFrames})')
            fullData_binary_chunks.append(B_extracted_numFrames)
        else:
            print(f'        Chunk_{ch}: Number of intact frames read ({numIntactFramesReadPerChunk}) did not match with number of frames requested ({args.numFrames})')
            sys.stderr.write('      Not included in the fullData. Rerun the measurement script if necessary.')

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
