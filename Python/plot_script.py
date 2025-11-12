import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn
mpl.use('Qt5Agg')

# to extend the arrays for the corresponding chip type
class plot_pdShank(object):

    def __init__(self, *args):
        pass

    def extend_all_data(self, chipType: 'int', pixelArray, numIntactFramesReadAllChunks, fullData_resetReadings_sum, fullData_pixReadings_sum, fullData_deltaSampled_sum, fullData_resetReadings, fullData_pixReadings, fullData_deltaSampled, numPixels):

        if chipType == 1:
            fullData_deltaSampled_sum_extended = fullData_deltaSampled_sum
            fullData_resetReadings_sum_extended = fullData_resetReadings_sum
            fullData_pixReadings_sum_extended = fullData_pixReadings_sum
            fullData_deltaSampled_extended = fullData_deltaSampled
            fullData_resetReadings_extended = fullData_resetReadings
            fullData_pixReadings_extended = fullData_pixReadings
            numPixels_extended = numPixels
            pass
        elif chipType == 0:  # if we have 400x10 chip, fill the gaps with zeros before plotting
            # for 2D graph
            fullData_deltaSampled_sum_splitted = np.vsplit(fullData_deltaSampled_sum, pixelArray[0]/pixelArray[1])
            zerosSplitted = np.vsplit(np.zeros((pixelArray[0], pixelArray[1])), pixelArray[0]/pixelArray[1])
            combined = [None] * (len(fullData_deltaSampled_sum_splitted) + len(zerosSplitted))
            combined[::2] = fullData_deltaSampled_sum_splitted; combined[1::2] = zerosSplitted
            fullData_deltaSampled_sum_extended = np.concatenate(combined, axis=0)

            fullData_pixReadings_sum_splitted = np.vsplit(fullData_pixReadings_sum, pixelArray[0] / pixelArray[1])
            combined[::2] = fullData_pixReadings_sum_splitted; combined[1::2] = zerosSplitted
            fullData_pixReadings_sum_extended = np.concatenate(combined, axis=0)

            fullData_resetReadings_sum_splitted = np.vsplit(fullData_resetReadings_sum, pixelArray[0] / pixelArray[1])
            combined[::2] = fullData_resetReadings_sum_splitted; combined[1::2] = zerosSplitted
            fullData_resetReadings_sum_extended = np.concatenate(combined, axis=0)

            # for linear graphs
            fullData_pixReadings_splitted = np.hsplit(fullData_pixReadings, pixelArray[0]/pixelArray[1])
            fullData_resetReadings_splitted = np.hsplit(fullData_resetReadings, pixelArray[0]/pixelArray[1])
            fullData_deltaSampled_splitted = np.hsplit(fullData_deltaSampled, pixelArray[0]/pixelArray[1])            
            zerosSplitted2 = np.hsplit(np.nan*np.ones((numIntactFramesReadAllChunks, pixelArray[0], pixelArray[1])), pixelArray[0]/pixelArray[1])

            combined = [None] * (len(fullData_pixReadings_splitted) + len(zerosSplitted2))
            combined[::2] = fullData_pixReadings_splitted; combined[1::2] = zerosSplitted2
            fullData_pixReadings_extended = np.concatenate(combined, axis=1)

            combined[::2] = fullData_resetReadings_splitted; combined[1::2] = zerosSplitted2
            fullData_resetReadings_extended = np.concatenate(combined, axis=1)
            
            combined[::2] = fullData_deltaSampled_splitted; combined[1::2] = zerosSplitted2
            fullData_deltaSampled_extended = np.concatenate(combined, axis=1)
            
            numPixels_extended = numPixels*2

        all_extended_data = {'fullData_deltaSampled_sum_extended': fullData_deltaSampled_sum_extended,
                             'fullData_pixReadings_sum_extended': fullData_pixReadings_sum_extended,
                             'fullData_resetReadings_sum_extended' : fullData_resetReadings_sum_extended,
                             'fullData_deltaSampled_extended' : fullData_deltaSampled_extended,
                             'fullData_pixReadings_extended' : fullData_pixReadings_extended,
                             'fullData_resetReadings_extended' : fullData_resetReadings_extended,
                             'numPixels_extended' : numPixels_extended}
        return all_extended_data

    def plot_2D(self, fullData_pixReadings_sum_extended, fullData_resetReadings_sum_extended, fullData_deltaSampled_sum_extended, numPixels_extended):
        plt.close(10)
        fig = plt.figure(10, figsize=(11, 9))
        plt.get_current_fig_manager().window.move(20, 20)  # move the window
        ax = fig.add_subplot(411)
        ax.set_title('2D Image - Pixel Readings', fontsize=17) #vmin=0, vmax=np.max(fullData_pixReadings_sum_extended), 
        seaborn.heatmap(np.transpose(fullData_pixReadings_sum_extended), vmin=np.min(fullData_pixReadings_sum_extended), vmax=np.max(fullData_resetReadings_sum_extended), cmap='mako', cbar_kws={"shrink": 0.7})  # robust = True, square=True, cmap='cividis', linewidth=0.5 is a parameter that we can use when bin multiple pixels
        #ax.set_aspect(17)
        ax.set_xlabel('Position Along Imager (mm)', fontsize=12)
        ax.set_ylabel('Position Along y Dir. (μm)', fontsize=12)
        x_pixels = np.array(range(0, int(numPixels_extended/10+1), int(numPixels_extended/80)))  # since 800x10 is our max, and 10 is constant
        x_distances = np.around(x_pixels * 0.00784, 1)  # in mm
        ax.set_xticks(x_pixels, x_distances.astype('str'), rotation=0, fontsize=12)
        y_pixels = np.array(range(0, 11, 5))
        y_distances = np.around(np.flip(y_pixels) * 7.84, 1)  # in um
        ax.set_yticks(y_pixels, y_distances.astype('str'), fontsize=12)

        ax = fig.add_subplot(412)
        ax.set_title('2D Image - Reset Readings', fontsize=17) #vmin=0, vmax=np.max(fullData_pixReadings_sum_extended), 
        seaborn.heatmap(np.transpose(fullData_resetReadings_sum_extended), vmin=np.min(fullData_pixReadings_sum_extended), vmax=np.max(fullData_resetReadings_sum_extended), cmap='mako', cbar_kws={"shrink": 0.7})  # robust = True, square=True, cmap='cividis', linewidth=0.5 is a parameter that we can use when bin multiple pixels
        #ax.set_aspect(17)
        ax.set_xlabel('Position Along Imager (mm)', fontsize=12)
        ax.set_ylabel('Position Along y Dir. (μm)', fontsize=12)
        x_pixels = np.array(range(0, int(numPixels_extended/10+1), int(numPixels_extended/80)))  # since 800x10 is our max, and 10 is constant
        x_distances = np.around(x_pixels * 0.00784, 1)  # in mm
        ax.set_xticks(x_pixels, x_distances.astype('str'), rotation=0, fontsize=12)
        y_pixels = np.array(range(0, 11, 5))
        y_distances = np.around(np.flip(y_pixels) * 7.84, 1)  # in um
        ax.set_yticks(y_pixels, y_distances.astype('str'), fontsize=12)

        ax = fig.add_subplot(413)
        ax.set_title('2D Image - Delta Sampled', fontsize=17) #vmin=0, 
        seaborn.heatmap(np.transpose(fullData_deltaSampled_sum_extended), vmin=0, cmap='mako', cbar_kws={"shrink": 0.7})  # robust = True, square=True, cmap='cividis', linewidth=0.5 is a parameter that we can use when bin multiple pixels
        #ax.set_aspect(17)
        ax.set_xlabel('Position Along Imager (mm)', fontsize=12)
        ax.set_ylabel('Position Along y Dir. (μm)', fontsize=12)
        x_pixels = np.array(range(0, int(numPixels_extended/10+1), int(numPixels_extended/80)))  # since 800x10 is our max, and 10 is constant
        x_distances = np.around(x_pixels * 0.00784, 1)  # in mm
        ax.set_xticks(x_pixels, x_distances.astype('str'), rotation=0, fontsize=12)
        y_pixels = np.array(range(0, 11, 5))
        y_distances = np.around(np.flip(y_pixels) * 7.84, 1)  # in um
        ax.set_yticks(y_pixels, y_distances.astype('str'), fontsize=12)
        
        ax = fig.add_subplot(414)
        ax.set_title('Column Lineplots - Delta Sampled', fontsize=17) 
        seaborn.lineplot(fullData_deltaSampled_sum_extended, legend='auto', picker=10)
        ax.set_xlim(0, numPixels_extended/fullData_deltaSampled_sum_extended.shape[1]-1)
        #ax.plot(np.arange(0, numPixels_extended/fullData_deltaSampled_sum_extended.shape[1]), fullData_deltaSampled_sum_extended, picker=10)
        #ax.set_aspect(17)
        ax.set_xlabel('Position Along Imager (mm)', fontsize=12)
        ax.set_ylabel('Photon Count', fontsize=12)
        x_pixels = np.array(range(0, int(numPixels_extended/10+1), int(numPixels_extended/80)))  # since 800x10 is our max, and 10 is constant
        x_distances = np.around(x_pixels * 0.00784, 1)  # in mm
        ax.set_xticks(x_pixels, x_distances.astype('str'), rotation=0, fontsize=12)
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0, fontsize=6)
        
        plt.tight_layout()
        bBox = list(ax.get_position().bounds); bBox[2] = bBox[2]*0.8; ax.set_position(bBox)
        
        fig.canvas.mpl_connect('button_press_event', self.onclick)
        
        plt.show()
        
        return fig
    
    def plot_3D(self, fullData_deltaSampled_sum_extended, numPixels_extended):
        plt.close(28)
        fig = plt.figure(28, figsize=(11, 4.55))
        plt.get_current_fig_manager().window.move(20, 10)  # move the window
        ax = plt.axes(projection='3d')
        ax.set_title('3D Image - Delta Sampled', fontsize=17) #vmin=0, vmax=np.max(fullData_pixReadings_sum_extended), 
        X, Y = np.meshgrid(np.array(range(0, int(numPixels_extended/10)))*0.00784, np.array(range(0, 10))*7.84)
        Z = np.flip(np.transpose(fullData_deltaSampled_sum_extended), axis=0)
        ax.plot_surface(X, Y, Z, cmap='mako', antialiased=False, rstride=1, cstride=1, edgecolor='darkslateblue', linewidth=0.1, vmin=0)
        ax.set_xlabel('Position Along Imager (mm)', fontsize=12)
        ax.set_ylabel('Pos. Along y Dir. (μm)', fontsize=12)
        x_distances = np.around(np.array(range(0, int(numPixels_extended/10+1), int(numPixels_extended/20)))*0.00784, 1)  # in mm --- since 800x10 is our max, and 10 is constant
        ax.set_xticks(x_distances, x_distances.astype('str'), fontsize=12)
        y_distances = np.around(np.flip(np.array(range(0, 11, 5))) * 7.84, 1)  # in um
        ax.set_yticks(y_distances, y_distances.astype('str'), fontsize=12)
        fig.set_facecolor('lightsteelblue'); ax.set_facecolor('lightsteelblue') 
        
        ax.view_init(20, 250)
        
        #plt.tight_layout()
        fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show()
        
        #seaborn.set_theme(style='white')
        
        return fig

    def plot_linear_graphs(self, numIntactFramesReadAllChunks, numPixels_extended, fullData_pixReadings_extended, fullData_resetReadings_extended, fullData_deltaSampled_sum_extended):
        plt.close(17)
        fig = plt.figure(17, figsize=(6.7, 7.5))
        plt.get_current_fig_manager().window.move(1130, 20)  # move the window

        ax = fig.add_subplot(411)
        ax.set_title('All Frames pixReadings Flattened to check for saturation - Scatter Plot')
        for k in range(numIntactFramesReadAllChunks):
            ax.scatter(np.arange(0, numPixels_extended), fullData_pixReadings_extended.reshape((numIntactFramesReadAllChunks, numPixels_extended))[k, :], picker=1)
        plt.xlim([0, numPixels_extended])
        plt.ylim([0, 2 ** 10])
        # plt.xticks(np.arange(0, numPixels_extended + 1, step=1000))
        # plt.yticks(np.arange(0, 2 ** 10 + 1, step=512))

        ax = fig.add_subplot(412)
        ax.set_title('All Frames resetReadings Flattened - Scatter Plot')
        for k in range(numIntactFramesReadAllChunks):
            ax.scatter(np.arange(0, numPixels_extended), fullData_resetReadings_extended.reshape((numIntactFramesReadAllChunks, numPixels_extended))[k, :], picker=1)
        plt.xlim([0, numPixels_extended])
        plt.ylim([0, 2 ** 10])
        # plt.xticks(np.arange(0, numPixels_extended + 1, step=1000))
        # plt.yticks(np.arange(0, 2 ** 10 + 1, step=512))

        ax = fig.add_subplot(413)
        ax.set_title('All Delta Sampled Frames Accumulated - semilogy')
        ax.semilogy(np.arange(0, numPixels_extended), fullData_deltaSampled_sum_extended.flatten(), color='red', picker=1)
        plt.fill_between(np.arange(0, numPixels_extended), fullData_deltaSampled_sum_extended.flatten(), color='red', alpha=0.62)
        plt.xlim([0, numPixels_extended])

        ax = fig.add_subplot(414)
        ax.set_title('All Delta Sampled Frames Accumulated - linear')
        ax.plot(np.arange(0, numPixels_extended), fullData_deltaSampled_sum_extended.flatten(), picker=10)
        plt.xlim([0, numPixels_extended])
        plt.xticks(np.arange(0, numPixels_extended + 1, step=1000))

        plt.tight_layout()

        fig.canvas.mpl_connect('button_press_event', self.onclick)

        plt.show()
        #plt.show(block=False)
        #plt.pause(5)

        return fig
    
    def onclick(self, event):
        print('%s click: button=%d, xdata=%f, ydata=%f' % ('double' if event.dblclick else 'single', event.button, event.xdata, event.ydata))
