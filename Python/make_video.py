import numpy as np
import plot_script
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def simple_animate_figures(figure_list, interval=500):
    """
    Animate a list of pre-saved matplotlib figures
    Works with modern matplotlib versions
    
    Parameters:
    figure_list: list of matplotlib figure objects
    interval: time between frames in milliseconds
    """
    
    # Convert figures to image arrays
    images = []
    for fig in figure_list:
        # Draw the figure
        fig.canvas.draw()
        
        # Get the image as RGB array - modern matplotlib method
        buf = fig.canvas.buffer_rgba()
        img_array = np.asarray(buf)
        
        # Convert RGBA to RGB (remove alpha channel)
        img_rgb = img_array[:, :, :3]
        images.append(img_rgb)
    
    # Create animation figure
    anim_fig, ax = plt.subplots(figsize=(12, 8))
    ax.axis('off')  # Hide axes
    
    # Display first image
    im = ax.imshow(images[0])
    
    def animate_frame(frame_num):
        im.set_array(images[frame_num])
        ax.set_title(f'Frame {frame_num + 1}/{len(images)}', fontsize=14, pad=10)
        return [im]
    
    # Create animation
    anim = animation.FuncAnimation(anim_fig, animate_frame, frames=len(images),
                                 interval=interval, repeat=True, blit=True)
    
    return anim_fig, anim

filename = '/users/syilmaz/Python/python_pdshank/savedata/20241228_metasurface_test/with_metasurface_video_v2/v4_intlog_gtx_rem_2.92_metasurface_video_v2_z66-50_to_166-50.npz'
ps = plot_script.plot_pdShank()

all_extended_data = np.load(filename,allow_pickle=True)['all_extended_data'].tolist()
numFrames = np.load(filename,allow_pickle=True)['numFrames'].tolist()
downsamplingRatio = np.load(filename,allow_pickle=True)['downsamplingRatio'].tolist()

fullData_deltaSampled_extended = all_extended_data['fullData_deltaSampled_extended']
fullData_deltaSampled_extended = fullData_deltaSampled_extended.reshape((downsamplingRatio,int(numFrames/downsamplingRatio),200,10),order='F')
fullData_deltaSampled_sum_extended = np.sum(fullData_deltaSampled_extended, axis=0)
fullData_deltaSampled_sum_extended[np.isnan(fullData_deltaSampled_sum_extended)] = 0

ims = []
for i in range(int(numFrames/downsamplingRatio)):
    fullData_deltaSampled_sum_extended[i,:,:]
    
    fig10 = ps.plot_2D(all_extended_data['fullData_pixReadings_sum_extended'], all_extended_data['fullData_resetReadings_sum_extended'], fullData_deltaSampled_sum_extended[i,:,:], all_extended_data['numPixels_extended'])
    ims.append(fig10)

anim_fig, animation_obj = simple_animate_figures(ims, interval=22.2935)
animation_obj.save('v4_intlog_gtx_rem_2.92_metasurface_video_v2_z66-50_to_166-50_fps44-856.gif', writer='pillow', fps=44.856)
plt.show()
