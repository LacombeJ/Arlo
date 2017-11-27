config = {}

config['hdf5_file'] = 'input.hdf5'  # hdf5 file with Fuel format

config['max_frames'] = 1067 # max files
config['train_data_size'] = config['max_frames']

# parameters of MDN
config['components_size'] = 50
config['sampling_bias'] = 1.0
config['seed'] = 66478
