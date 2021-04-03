import argparse
import csv
import math
import numpy as np
import os
import subprocess
import time
from dotmap import DotMap

from verifai.samplers.scenic_sampler import ScenicSampler
from verifai.scenic_server import ScenicServer
from verifai.falsifier import generic_falsifier, generic_parallel_falsifier
from verifai.monitor import multi_objective_monitor

class ADE_FDE(multi_objective_monitor):
	def __init__(self, model_path):
		self.model_path = model_path
		self.num_objectives = 2
		priority_graph = None

		def specification(traj):
			i = 10  # FIXME: separates historical trajectories
			hist, truth = [xy[-1] for xy in traj[:i]], [xy[-1] for xy in traj[i:]]

			# process historical trajectory csv file
			with open(f"{model_path}/test_obs/data/0.csv", 'w', newline='') as csvfile:
		        writer = csv.writer(csvfile)
		        writer.writerow(['TIMESTAMP', 'TRACK_ID', 'OBJECT_TYPE', 'X', 'Y', 'CITY_NAME'])
		        track_id = '00000000-0000-0000-0000-000000000000'
		        for timestamp, agent_traj in enumerate(hist):
		            writer.writerow([timestamp, track_id, 'AV', agent_traj[0], agent_traj[1], 'N/A'])
		        f.close()

			# run behavior prediction model
			os.chdir(model_path)
			subprocess.run(['python', 'preprocess_data.py', '-n', '1'])
			subprocess.run(['python', 'test.py', '-m', 'lanegcn', '--weight=/absolute/path/to/36.000.ckpt', '--split=test', '--carla_map_path=/absolute/path/to/map'])

			# extract predicted trajectories from csv file
			preds = np.genfromtxt(f'{model_path}/results/lanegcn/0.csv', delimiter=',', skip_header=1)
			pred_len = preds.shape[0]
			truth = truth[:pred_len]

			# compute objectives
			ADE = float(
				sum(
					math.sqrt(
						(preds[i, 0] - truth[i, 0]) ** 2
						+ (preds[i, 1] - truth[i, 1]) ** 2
					)
					for i in range(pred_len)
				)
				/ pred_len
			)
			FDE = math.sqrt(
				(preds[-1, 0] - truth[-1, 0]) ** 2
				+ (preds[-1, 1] - truth[-1, 1]) ** 2
			)

			rho = (ADE, FDE)
			return rho

		super().__init__(specification, priority_graph)

def run_experiment(path, monitor, parallel=False):
	sampler = ScenicSampler.fromScenario(path)
	falsifier_params = DotMap(
		n_iters=None,
		save_error_table=True,
		save_safe_table=True,
		max_time=60
	)
	server_options = DotMap(maxSteps=100, verbosity=0)
	
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
	parser.add_argument('--path', '-p', type=str, default='intersection/intersection_01.scenic', help='Path to Scenic script')
	parser.add_argument('--parallel', action='store_true')
	parser.add_argument('--model', '-m', type=str, default='', help='Path to behavior prediction model')
	args = parser.parse_args()

	monitor = ADE_FDE(args.model)
	falsifier = run_experiment(args.path, args.parallel, monitor)
