import os
import shutil
import glob
import getopt
import sys

from pathlib import Path
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from xml.dom.minidom import parseString
import dicttoxml as d2x
import pylab as pl

from tracking_params_generator import *


if sys.platform == 'win32':
	binary_extension = '.exe'
else:
	binary_extension = ''

def create_xml_property(xml_output, params):
	
	xml = d2x.dicttoxml(params, attr_type=False)
	dom = parseString(xml)
	xml_str = dom.toprettyxml(indent="  ")
	with open(xml_output, "w") as f:
		f.write(xml_str)
	
	return 

def get_tracking_params():
	F = np.array([[1,1],[0,1]])
	H = np.array([[1,0]])
	
	s = 0.01
	R = np.array([[s*s]])
	
	Q = np.array([[5e-6, 7e-7], [7e-7, 6e-6]])

	P = np.array([[s*s, 0], [0, s*s]])
	Xk_ = np.array([[0], [0]])
	
	residual = 100000
	points = 2
	velocity = 220
	alpha = 0.3
	beta = 0.1
	
	IdentifierGenerator = AllTragetIdentifierParamsGenerator(residual)
	
	#FilterGenerator = AlphaBetaParamsGenerator(alpha, beta)
	FilterGenerator = KalmanParamsGenerator(F, H, Q, R, P, Xk_)
	#FilterGenerator = DynamicKalmanParamsGenerator(F, H, Q, R, P, Xk_, velocity)
	InitializerGenerator = PointsInitializerParamsGenerator(points)
	ExtrapolatorGenerator = LinearExtrapolatorParamsGenerator()
	
	Generator = ParamsGenerator(IdentifierGenerator, FilterGenerator, InitializerGenerator, ExtrapolatorGenerator)
	
	params = Generator.get_params()
	
	return params
	
def get_data_from_csv(input_file):
	data = np.empty((0))
	with open(input_file) as file:
		new_file = True
		for line in file:
			# Skip first head line
			if new_file:
				new_file = False
				continue
			
			line = line.rstrip()
			current_line = line.split(";")
			angle = float(current_line[1])
			data = np.append(data, angle)

	return data

def get_me_mse(input_file, reference):
	me = np.empty((0))
	mse = np.empty((0))
	
	data = get_data_from_csv(input_file)
	data_ref = get_data_from_csv(reference)
	
	data_size = len(data_ref)
	if len(data) != data_size:
		print("len(data) != data_size")
		exit()
	
	# Note: 2 first point used to filter initializing
	for i in range(0, data_size):
		me_tmp = (data[i] - data_ref[i])
		me = np.append(me, me_tmp)
		mse = np.append(mse, me_tmp ** 2)
	
	# Note: Pandas is very slow shit
	# data = pd.read_csv(input_file, sep=';')
	# data_ref = pd.read_csv(reference, sep=';')
	# data_size = len(data_ref.index)
	
	# if len(data.index) != data_size:
		# print("len(data.index) != len(data_ref.index)")
		# exit()
		
	# for i in range(0, data_size):
		# me = np.append(me, (data['OBJ_1'][i] - data_ref['OBJ_1'][i]))
		# mse = np.append(mse, (me[i] ** 2))

	return me, mse


def calculate_statistics(path_to_data, reference):
	files_tracked = glob.glob(os.path.join(path_to_data, "*.csv"))
	if len(files_tracked) == 0:
		print("len(files_tracked) = 0")
		exit()	

	me = np.empty((0))
	mse = np.empty((0))
	
	if len(files_tracked) == 1:
		file = files_tracked[0]
		me, mse = get_me_mse(file, reference)
		
	else:
		me, mse = get_me_mse(files_tracked.pop(0), reference)
		files_count = 1
		for file in files_tracked:
			tmp_me, tmp_mse = get_me_mse(file, reference)
			me = me + tmp_me
			mse = mse + tmp_mse
			files_count += 1
		
		me /= files_count
		mse /= files_count
		
	return me, mse
	
	
def write_csv(path, file):
	dataf = pd.read_csv(file, sep="\t", index_col=False)
	needed_data = dataf[['T_m','T1','cos_v']]
	filename = Path(file)
	csv_output = filename.with_suffix('.csv')
	needed_data.to_csv(csv_output, sep=";", index=False, float_format='%.6f')

	
def run_tracking(input_path, output_path, properties):
	exe = os.path.join('bin', 'tracking_test' + binary_extension)
	if not os.path.exists(exe):
		print(exe, "does not exist")
		exit()
		
	os.system(exe + " -i " + input_path + " -o " + output_path + " -p " + properties)

	
def get_info(path):
	prepared_path = ""
	distance = ""
	
	if os.path.isdir(path):
		distance = os.path.basename(path)
		prepared_path = path
		
	elif os.path.isfile(path):
		prepared_path = os.path.dirname(path)
		distance = os.path.basename(prepared_path)
	
	else:
		print(path, 'does not exist')
		exit()
	
	return prepared_path, distance

	
def plot_graphics(params, path_to_data, output_path=None, me=None, mse=None):
	#pictures_dpi = 1200
	pictures_dpi = 600
	tracking_type = params['tracking_object']['Filter']['type']
	data_path, distance = get_info(path_to_data)
	if output_path != None and not os.listdir(output_path):
		print(output_path, 'is empty')
		exit()
	
	img_path = os.path.join(data_path, '..', 'img')
	if not os.path.exists(img_path):
		os.makedirs(img_path)
	
	if output_path != None:
		files_tracked = glob.glob(os.path.join(output_path, "*.csv"))
		tracked_trajectory_file = files_tracked[0]
		df_tracked_traj = pd.read_csv(tracked_trajectory_file, sep=';')
	
	files_noised = glob.glob(os.path.join(data_path, "*.csv"))
	noised_trajectory_file = files_noised[0]
	df_noised_traj = pd.read_csv(noised_trajectory_file, sep=';')
	
	me_mean = me.mean()
	mse_mean = mse.mean()
		
	plt.figure()
	plt.title(tracking_type + ' ' + distance + ', MSE Mean = ' + "{0:.4e}".format(mse_mean))
	plt.xlabel('time [s]', fontsize=14)
	plt.ylabel('cos(phi) [rad]', fontsize=14)
	if output_path != None:
		plt.plot(df_tracked_traj.Time, df_tracked_traj.OBJ_1, color='red', label='estimated traj')
	plt.plot(df_noised_traj.T_m, df_noised_traj.cos_v, color='b', linewidth=7.0, alpha=0.3, label='noised traj')
	plt.grid(True, linewidth=0.3)
	plt.legend(fancybox=True, loc=1)
	plt.tick_params(labelsize=12)
	plt.savefig(os.path.join(img_path, tracking_type + '_' + distance + '_traj.png'), bbox_inches='tight', dpi = pictures_dpi)
	
	if me.any() and mse.any():
		print("Plotting.....")
		df_me = pd.DataFrame(data=me, columns=['err'])	
		df_mse = pd.DataFrame(data=mse, columns=['err'])
		plt.figure()
		plt.title(tracking_type + ' ' + distance + ' ME, ' + 'Mean = ' + "{0:.4e}".format(me_mean))
		plt.xlabel('time [s]', fontsize=14)
		plt.ylabel('cos(phi) [rad]', fontsize=14)
		plt.plot(df_me.index.values, df_me.err)
		plt.grid(True, linewidth=0.3)
		plt.tick_params(labelsize=12)
		plt.savefig(os.path.join(img_path, tracking_type + '_' + distance + '_me.png'), bbox_inches='tight', dpi = pictures_dpi)
		plt.yscale('log')
		
		plt.figure()
		plt.title(tracking_type + ' ' + distance + ' MSE, ' + 'Mean = ' + "{0:.4e}".format(mse_mean))
		plt.xlabel('time [s]', fontsize=14)
		plt.ylabel('$cos(phi)^2$ [rad]', fontsize=14)
		plt.plot(df_mse.index.values, df_mse.err)
		plt.grid(True, linewidth=0.3)
		plt.tick_params(labelsize=12)
		plt.savefig(os.path.join(img_path, tracking_type + '_' + distance + '_mse.png'), bbox_inches='tight', dpi = pictures_dpi)

		plt.figure()
		plt.title('log sc ' + tracking_type + ' ' + distance + ' MSE, ' + 'Mean = ' + "{0:.4e}".format(mse_mean))
		plt.xlabel('time [s]', fontsize=14)
		plt.ylabel('$cos(phi)^2$ [rad]', fontsize=14)
		plt.plot(df_mse.index.values, df_mse.err)
		plt.grid(True, linewidth=0.3)
		plt.tick_params(labelsize=12)
		plt.yscale('log')
		plt.savefig(os.path.join(img_path, tracking_type + '_' + distance + '_mse_log.png'), bbox_inches='tight', dpi = pictures_dpi)
		
	plt.show()

	
def usage():
	print("Keys:")
	print("\t-i(--input) REQUIRED: input file or path to files")
	print("\t-p: generate tracking params xml. Note: You can manualy create  \"tracking_property_default.xml\" and use it without this key")
	print("\t-g: plot graphics")
	print("\t-h(--help): this help")
	
	
def main():
	path_to_data = ""
	properties_file = os.path.join('bin', 'tracking_property_default.xml')
	need_create_properties = False
	need_graphics = False
	need_find_optimal_Q = False
	
	try:
		opts, args = getopt.getopt(sys.argv[1:], "i:pgh", ["input=", "help"])
	except getopt.GetoptError as err:
		usage()
		sys.exit(2)
	
	if len(opts) < 1:
		print("Please, enter data path or file with -i key")
		usage()
		sys.exit(2)
		
	for o, a in opts:
		if o in ("-i", "--input"):
			path_to_data = a
		elif o in ("-p"):
			need_create_properties = True
		elif o in ("-g"):
			need_graphics = True		
		elif o in ("-h", "--help"):
			usage()
		else:
			assert False, "unhandled option"
	
	data_path, distance = get_info(path_to_data)
	
	reference = os.path.join(data_path, 'reference', 'references.csv')
	if not os.path.isfile(reference):
		print(reference, "does not exist")
		exit()
	
	output_path = os.path.join(data_path, 'tracking')
	if os.path.exists(output_path):
		shutil.rmtree(output_path)
	os.makedirs(output_path)
	
	params = {}

	if need_create_properties:
		properties_file = os.path.join('bin', 'tracking_property.xml')
		params = get_tracking_params()
		create_xml_property(properties_file, params)
	
	run_tracking(path_to_data, output_path, properties_file)
	me, mse = calculate_statistics(output_path, reference)

	if need_graphics:
		plot_graphics(params, path_to_data, output_path, me, mse)


if __name__ == "__main__":
	main()
