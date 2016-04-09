clear all
rosshutdown;

name = '/adaptation_server';
rosinit;
node = robotics.ros.Node(name);
server = robotics.ros.ServiceServer(node,['/baxter_adapt', name], 'baxter_adapt/Adaptation');
server.NewRequestFcn = @adaptationCallback;

global pool;
pool = gcp;

global weights;
weights = [];

