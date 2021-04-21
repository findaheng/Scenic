import argparse
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import subprocess
import time
from dotmap import DotMap

from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.scenic_server import ScenicServer
from verifai.falsifier import generic_falsifier, generic_parallel_falsifier
from verifai.monitor import multi_objective_monitor

class ADE_FDE(multi_objective_monitor):
    def __init__(self, model_path, timepoint=20, debug=False):
        self.model_path = model_path
        self.num_objectives = 2
        self.timepoint = timepoint; assert timepoint >= 20, 'Must allow at least 20 timesteps of past trajectories!'
        self.debug = debug
        priority_graph = None

        def specification(traj):
            hist_traj = traj[timepoint-20:timepoint]
            gt_traj = traj[timepoint:timepoint+15]
            gts = np.asarray([(tj[-1][0], tj[-1][1]) for tj in gt_traj])
            gt_len = len(gts)

            if self.debug:
                plt.plot([gt[-1][0] for gt in traj], [gt[-1][1] for gt in traj], color='black')
                plt.plot([gt[-1][0] for gt in hist_traj], [gt[-1][1] for gt in hist_traj], color='blue')
                plt.plot([gt[-1][0] for gt in gt_traj], [gt[-1][1] for gt in gt_traj], color='yellow')

            # process historical trajectory csv file
            with open(f"{model_path}/dataset/test_obs/data/0.csv", 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['TIMESTAMP', 'TRACK_ID', 'OBJECT_TYPE', 'X', 'Y', 'CITY_NAME'])
                track_id = '00000000-0000-0000-0000-000000000000'
                for timestamp in range(timepoint-20, timepoint):
                    for obj_type, agent_pos in enumerate(traj[timestamp]):
                        if obj_type == 0:
                            obj_type = 'AV'
                        elif obj_type == num_agents - 1:
                            obj_type = 'AGENT'
                        else:
                            obj_type = 'OTHER'
                        writer.writerow([timestamp, track_id, obj_type, agent_pos[0], agent_pos[1], 'N/A'])
            csvfile.close()

            # run behavior prediction model
            os.chdir(model_path)
            subprocess.run(['python', 'preprocess_data.py', '-n', '1'])
            subprocess.run(['python', 'test.py', '-m', 'lanegcn', f'--weight={model_path}/36.000.ckpt', '--split=test', '--map_path=/home/carla_challenge/Desktop/francis/Scenic/tests/formats/opendrive/maps/CARLA/Town05.xodr'])

            ADEs, FDEs = [], []
            for i in range(6):
                if self.debug:
                    p = pd.read_csv(f'{model_path}/results/lanegcn/predictions_{i}_0.csv')
                    plt.plot(p['X'], p['Y'], color='green')

                # extract predicted trajectories from csv file
                preds = np.genfromtxt(f'{model_path}/results/lanegcn/predictions_{i}_0.csv', delimiter=',', skip_header=1)
                pred_len = preds.shape[0]
                if gt_len < pred_len:
                    preds = preds[:gt_len]

                # compute objectives
                ADE = float(
                    sum(
                        math.sqrt(
                            (preds[i, 0] - gts[i, 0]) ** 2
                            + (preds[i, 1] - gts[i, 1]) ** 2
                        )
                        for i in range(min(pred_len, gt_len))
                    )
                    / pred_len
                )
                FDE = math.sqrt(
                    (preds[-1, 0] - gts[-1, 0]) ** 2
                    + (preds[-1, 1] - gts[-1, 1]) ** 2
                )

                ADEs.append(ADE)
                FDEs.append(FDE)
                print(f'ADE: {ADE}, FDE: {FDE}')

            minADE, minFDE = min(ADEs), min(FDEs)
            rho = (2-minADE, 2-minFDE)

            plt.show()
            return rho

        super().__init__(specification, priority_graph)

def announce(message):
    m = f'* {message} *'
    border = '*' * len(m)
    print(border)
    print(m)
    print(border)

def run_experiment(path, monitor, parallel=False):
    sampler = ScenicSampler.fromScenario(path)
    falsifier_params = DotMap(
        n_iters=None,
        save_error_table=True,
        save_safe_table=True,
        max_time=60
    )
    server_options = DotMap(maxSteps=300, verbosity=0)
    
    falsifier_cls = generic_parallel_falsifier if parallel else generic_falsifier
    falsifier = falsifier_cls(sampler=sampler, falsifier_params=falsifier_params,
                                    server_class=ScenicServer,
                                    server_options=server_options,
                                    monitor=monitor, scenic_path=path)
    t0 = time.time()
    falsifier.run_falsifier()
    t = time.time() - t0
    print(f'\nGenerated {len(falsifier.samples)} samples in {round(t, 3)} seconds with {falsifier.num_workers} worker(s)')
    print(f'Number of counterexamples: {len(falsifier.error_table.table)}')
    return falsifier

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', '-p', type=str, default='/home/carla_challenge/Desktop/francis/Scenic/examples/carla/Behavior_Prediction/intersection/intersection_01.scenic', help='Path to Scenic script')
    parser.add_argument('--parallel', action='store_true')
    parser.add_argument('--model', '-m', type=str, default='/home/carla_challenge/Desktop/francis/LaneGCN', help='Path to behavior prediction model')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    monitor = ADE_FDE(args.model, args.debug)
    falsifier = run_experiment(args.path, monitor, args.parallel)
