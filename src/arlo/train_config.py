config = {}

config['robot'] = 'al5d' #TODO remove all references of 'robot' and hardcode al5d support for now
config['task_to_perform'] = 0

config['hdf5_file'] = 'input.hdf5'  # hdf5 file with Fuel format

config['input_columns'] = ['task0']
config['output_columns'] = ['joint1', 'joint2', 'joint3', 'joint4','joint5','gripper','steps_to_goal']
config['future_predictions'] = [1]

config['max_frames'] = 2000 # max files

config['train_size'] = 0.80  # fraction of data that goes into train set

# Network parameters
#config['task_to_perform'] = 3006
config['batch_size'] = 128  # number of samples taken per each update
config['hidden_size'] = 100
config['learning_rate'] = .005
config['training_optimizer'] = 'RMSProp'  # Adam, RMSProp, Adagrad, Adadelta
config['learning_rate_decay'] = 0.5  # set to 0 to not decay learning rate
config['lr_decay_every_n_epochs'] = 100
config['decay_rate'] = 0.999  # decay rate for rmsprop and AdaDelta
config['step_clipping'] = 1.0  # clip norm of gradients at this value
config['dropout'] = .5
config['max_norm_threshold'] = 4
config['nepochs'] = 5  # number of full passes through the training data
config['seq_length'] = 50  # number of time-steps in the sequence (for truncated BPTT)
config['seq_redundancy'] = 50
config['linear_before_recurrent_size'] = 0
config['use_layer_norm'] = True

config['layer_models'] = ['lstm','lstm','lstm'] # feedforward, lstm, rnn
config['num_recurrent_layers'] = len(config['layer_models'])
config['plot_hidden_states'] = False

# parameters of cost function
config['cost_mode'] = 'RL-MDN'  # MDN, RL-MDN, MSE

# Reinforcement learning parameters
config['gamma'] = 1.0

# parameters of MDN
config['components_size'] = 50
config['sampling_bias'] = 1.0
config['seed'] = 66478

# parameters of vision
config['pixel_depth'] = 255
config['image_shape'] = (128, 128) # height, width
config['num_channels'] = 3

# parameters of multi-task learning : The idea is to train a network on data of multiple tasks. The ID if the task to be executed is given as an input in each time-step
config['multi_task_mode'] = 'ID'  # ID, goal
config['task_ID_type'] = 'one-hot' # feedforward, one-hot
#config['task_ID_FF_dims'] = [65, 20]
config['game_tasks'] = [0] # jonathan hardcoded (task 0)

# parameters of multi-timescale learning : The idea is to enable different layers of LSTM or RNN to work at different time-scales
config['layer_resolutions'] = [1, 1, 1]

# outputting one dimension at a time parameters
config['single_dim_out'] = True

# parameters about auxiliary predictions - The idea is to have some layers to predict another related prediction, for instance, predict the pose of object or the pose of gripper in next 4 timesteps
config['future_predictions'] = [1]
config['prediction_cost_weights'] = [1]

# We make a stacked RNN with the following skip connections added if the corresponding parameter is True
config['connect_x_to_h'] = True
config['connect_h_to_h'] = 'one-previous'  # all-previous , two-previous, one-previous
config['connect_h_to_o'] = True


# Save paths
config['save_path'] = 'models/model_save.pkl'
config['last_path'] = 'models/model_last.pkl'
config['load_path'] = config['save_path']




