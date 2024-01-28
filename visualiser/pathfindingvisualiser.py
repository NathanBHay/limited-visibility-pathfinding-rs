"""
This is a visualiser for pathfinding algorithms for more specific analysis of

This could be optimised in the future with a faster json parser, or a faster
visualisation library.
"""

from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import numpy as np
from json import load as json_load
from matplotlib.cm import viridis, Greys
import argparse
import time

viridis = viridis(np.linspace(0, 1, 256))
viridis[0, :] = 0  # Make the 0 value fully transparent
viridis = ListedColormap(viridis)
viridis1 = viridis(1.0)
node_size = 100
props = dict(boxstyle='round', facecolor='white',  alpha=0.5)  # Define properties for the text box

class Visualiser:
    def __init__(self, file_name: str) -> None:
        self.file_name = file_name
        self.figure, self.ax = plt.subplots()
        self.goal = None
        self.dims = (0, 0)
        with open(f'{self.file_name}_ground_truth.json') as f:
            ground_truth = json_load(f)
            self.dims = np.array(ground_truth['grid']).shape[::-1]
            self.ax.set_aspect('equal')

    def visualise_start_end(self):
        """
        Visualise the start and end points of the file_ground_truth.json file. Points in (x, y)
        notation.
        """
        with open(f'{self.file_name}_ground_truth.json') as f:
            ground_truth = json_load(f)
            if start := ground_truth['start']:
                self.ax.scatter(start[0] + 0.5, self.dims[0] - start[1] - 0.5, color=viridis1, s=node_size)
            self.goal = ground_truth['goal']
            if self.goal:
                self.ax.scatter(self.goal[0] + 0.5, self.dims[0] - self.goal[1] - 0.5, color=viridis1, s=node_size, marker='s')

    def visualise_ground_truth(self):
        """
        Visualise the file_ground_truth.json file. The ground truth is represented as a 2d bool array.
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Ground Truth', fontsize=16)
        with open(f'{self.file_name}_ground_truth.json') as f:
            ground_truth = json_load(f)
            ground_truth = np.array(ground_truth['grid']).astype(bool).transpose()
            self.ax.imshow(ground_truth, cmap='gray', extent=[0, self.dims[1], 0, self.dims[0]])

    def visualise_samplestar(self, iteration: int, labels: bool):
        """
        Visualise the file_sample_grid.json file. The sample grid itself is represented
        as a 2d array. The paths are just a series of points with a count of how many
        times they were visited. The current and next points follow basic (x, y) notation.
        The stats are a list of important stats added to the legend.
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Sample Grid at Iteration {iteration}', fontsize=16)
        with open(f'{self.file_name}_step_{iteration}.json') as f:
            sample_grid_obj = json_load(f)
            if labels:
                sample_grid = np.array(sample_grid_obj['sample_grid']).astype(float)
                for i in range(len(sample_grid)):
                    for j in range(len(sample_grid[i])):
                        self.ax.text(i, j, round(sample_grid[i, j], 3), ha="center", va="center", color=Greys(sample_grid[i, j]))

            paths = sample_grid_obj['paths']
            path_counts = np.zeros(self.dims)
            for path in paths:
                path_counts[path[0][1], path[0][0]] = path[1]
            self.ax.imshow(path_counts, cmap=viridis, interpolation='nearest', alpha=0.5, extent=[0, self.dims[1], 0, self.dims[0]])

            if self.goal:
                self.ax.scatter(self.goal[0] + 0.5, self.dims[0] - self.goal[1] - 0.5, color=viridis1, s=node_size, marker='s')

            if current := sample_grid_obj['current']:
                self.ax.scatter(current[0] + 0.5, self.dims[0] - current[1] - 0.5, color=viridis1, s=node_size)

            if next := sample_grid_obj['next']:
                marker = '*'
                if current := sample_grid_obj['current']:
                    diff = (next[0] - current[0], next[1] - current[1])
                    if diff == (0, 0): marker = 'o'
                    elif diff == (0, -1): marker = '^'
                    elif diff == (0, 1): marker = 'v'
                    elif diff == (1, 0): marker = '>'
                    elif diff == (-1, 0): marker = '<'
                self.ax.scatter(next[0] + 0.5, self.dims[0] - next[1] - 0.5, color=viridis1, s=node_size, marker=marker)

            if stats := sample_grid_obj['stats']:
                self.ax.text(1, 1, '\n'.join(stats), transform=self.ax.transAxes, fontsize=12, verticalalignment='top', horizontalalignment='right', bbox=props)

    def visualise_final_path(self):
        """
        Visualise the file_final_path.json file. The final path is represented as a edge list.
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Final Path', fontsize=16)
        self.visualise_start_end()
        with open(f'{self.file_name}_final_path.json') as f:
            final_path = json_load(f)
            for edge in final_path['path']:
                self.ax.plot([edge[0][0][0] + 0.5, edge[0][1][0] + 0.5], [self.dims[0] - edge[0][0][1] - 0.5, self.dims[0] - edge[0][1][1] - 0.5], c=viridis(edge[1]), linewidth=4)
            self.ax.text(1, 1, f'Path Len: {final_path["length"]}', transform=self.ax.transAxes, fontsize=14, verticalalignment='top', horizontalalignment='right', bbox=props)

    def visualise_all(self, labels: bool = True, limit: int = 1000):
        """
        Visualise all files
        """
        self.visualise_ground_truth()
        self.visualise_start_end()
        plt.savefig(f'{self.file_name}_ground_truth.png')
        self.ax.cla()
        for i in range(1, limit+1):
            self.visualise_ground_truth()
            try:
                self.visualise_samplestar(i, labels)
            except FileNotFoundError: break
            plt.savefig(f'{self.file_name}_step_{i}.png')
            self.ax.cla()
        self.ax.cla()
        self.visualise_ground_truth()
        self.visualise_final_path()
        plt.savefig(f'{self.file_name}_final_path.png')
        self.ax.cla()

def main():
    parser = argparse.ArgumentParser(description='Visualise pathfinding algorithms')
    parser.add_argument('file_name', type=str, help='The name of the file to visualise')
    parser.add_argument('-vs', '--visualise-specific', type=int, help='The specific step to visualise')
    parser.add_argument('-l', '--labels', type=bool, default=False, help='Whether to show labels on the sample grid')
    parser.add_argument('-i', '--limit', type=int, default=1000, help='The maximum number of steps to visualise')
    args = parser.parse_args()
    start_time = time.time()
    v = Visualiser(args.file_name)
    if args.visualise_specific:
        v.visualise_ground_truth()
        v.visualise_samplestar(args.visualise_specific, args.labels)
        plt.show()
    else:
        v.visualise_all(args.labels, args.limit)
    print(f'Time taken: {time.time() - start_time}s')

if __name__ == '__main__':
    main()
