# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
# OS (Operating system interfaces)
import os
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots

"""
Description:
    Initialization of constants.
"""


def main():
    """
    Description:
        ...
    """

    # Locate the path to the project folder.
    project_folder = os.getcwd().split('Object_Detection_Synthetic_Data')[0] + 'Object_Detection_Synthetic_Data'

    # Set the parameters for the scientific style.
    plt.style.use(['science'])

    # Convert the list to an array.
    metrics = np.array(metrics, dtype=np.float32)

    # Create a figure.
    fig, ax = plt.subplots(1, 1)

    # Display metrics data in a bar chart.
    for i, color_i in enumerate(['#bfdbd1', '#72837d', '#abcae4', '#88a1b6', '#667988', '#a64d79']):
        ax.bar(np.arange(0, 4, 1) + i*0.05, metrics[i, :], color=color_i, alpha=1.0, width = 0.05, 
               label=f'Type {i}')

    # Set parameters of the graph (plot).
    #   Set the x, y ticks.
    plt.xticks(np.arange(0, 4, 1) + 0.125, ['Precision', 'Recall', 'mAP@0.5', 'mAP@0.5:0.95'])
    plt.yticks(np.arange(0, 1.1, 0.1))
    #   Set the y limits.
    ax.set(ylim=[0.8, 1.1])
    #   Label
    ax.set_xlabel(r'Metrics', fontsize=15); ax.set_ylabel(r'Score', fontsize=15) 
    #   Set parameters of the visualization.
    ax.grid(which='major', linewidth = 0.25, linestyle = '--')
    # Get handles and labels for the legend.
    handles, labels = plt.gca().get_legend_handles_labels()
    # Remove duplicate labels.
    legend = dict(zip(labels, handles))
    # Show the labels (legends) of the graph.
    ax.legend(legend.values(), legend.keys(), fontsize=10.0)


if __name__ == '__main__':
    sys.exit(main())