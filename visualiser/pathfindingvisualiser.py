"""
This is a visualiser for pathfinding algorithms for more specific analysis of
"""

import matplotlib.pyplot as plt
import numpy as np
from json import load as json_load
from matplotlib.cm import viridis, Greys
import argparse

viridis1 = viridis(1.0)

class Visualiser:
    def __init__(self, file_name: str) -> None:
        self.file_name = file_name
        _, self.ax = plt.subplots()
        self.goal = None

    def visualise_start_end(self):
        """
        Visualise the start and end points of the file_ground_truth.json file
        """
        with open(f'{self.file_name}_ground_truth.json') as f:
            ground_truth = json_load(f)
            if start := ground_truth['start']:
                self.ax.scatter(start[0], start[1], color=viridis1, s=200)
            self.goal = ground_truth['goal']
            if self.goal:
                self.ax.scatter(self.goal[0], self.goal[1], color=viridis1, s=200, marker='s')

    def visualise_ground_truth(self):
        """
        Visualise the file_ground_truth.json file
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Ground Truth')
        with open(f'{self.file_name}_ground_truth.json') as f:
            ground_truth = json_load(f)
            ground_truth = np.array(ground_truth['grid']).astype(bool).transpose()
            self.ax.imshow(ground_truth, cmap='gray')

    def visualise_sample_grid(self, iteration: int, labels: bool = True):
        """
        Visualise the file_sample_grid.json file
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Sample Grid at Iteration {iteration}')
        with open(f'{self.file_name}_step_{iteration}.json') as f:
            sample_grid_obj = json_load(f)
            if labels:
                sample_grid = np.array(sample_grid_obj['sample_grid']).astype(float)
                for i in range(len(sample_grid)):
                    for j in range(len(sample_grid[i])):
                        self.ax.text(i, j, round(sample_grid[i, j], 3), ha="center", va="center", color=Greys(sample_grid[i, j]))

            paths = sample_grid_obj['paths']
            for path in paths:
                for p in path[1]:
                    self.ax.plot([path[0][0], p[0][0]], [path[0][1], p[0][1]], c=viridis(p[1]), linewidth=4)

            if self.goal:
                self.ax.scatter(self.goal[0], self.goal[1], color=viridis1, s=200, marker='s')

            if current := sample_grid_obj['current']:
                self.ax.scatter(current[0], current[1], color=viridis1, s=200)

            if next := sample_grid_obj['next']:
                marker = '*'
                if current := sample_grid_obj['current']:
                    diff = (next[0] - current[0], next[1] - current[1])
                    if diff == (0, 0): marker = 'o'
                    elif diff == (0, -1): marker = '^'
                    elif diff == (0, 1): marker = 'v'
                    elif diff == (1, 0): marker = '>'
                    elif diff == (-1, 0): marker = '<'
                self.ax.scatter(next[0], next[1], color=viridis1, s=200, marker=marker)

            if stats := sample_grid_obj['stats']:
                for stat in stats:
                    self.ax.text(stat[0], stat[1], ha="bottom", va="left", color=viridis1)

    def visualise_final_path(self):
        """
        Visualise the file_final_path.json file
        """
        self.ax.set_title(f'{self.file_name.capitalize()} Final Path')
        self.visualise_start_end()
        with open(f'{self.file_name}_final_path.json') as f:
            for path in json_load(f)['path']:
                for p in path[1]:
                    self.ax.plot([path[0][0], p[0][0]], [path[0][1], p[0][1]], c=viridis(p[1]), linewidth=4)

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
                self.visualise_sample_grid(i, labels)
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
    v = Visualiser(args.file_name)
    if args.visualise_specific:
        v.visualise_ground_truth()
        v.visualise_sample_grid(args.visualise_specific, args.labels)
        plt.show()
    else:
        v.visualise_all(args.labels, args.limit)

if __name__ == '__main__':
    main()
