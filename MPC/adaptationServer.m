clear all
rosshutdown;

name = '/adaptation_server';
rosinit;
node = robotics.ros.Node(name);
server = robotics.ros.ServiceServer(node,['/baxter_adapt', name], 'baxter_adapt/Adaptation');
server.NewRequestFcn = @adaptationCallback;

node_learning = robotics.ros.Node('/learning_server');
learning_server = robotics.ros.ServiceServer(node, '/baxter_adapt/learning_server','baxter_adapt/Learning');
learning_server.NewRequestFcn = @learningCallback;

global pool;
global weights;
global contexts;
global config;
global imitation_traj;
global adaptaion_traj;

pool = gcp;
weights = [];
contexts = [];
config = [];
imitation_traj = [];
adaptaion_traj = [];

