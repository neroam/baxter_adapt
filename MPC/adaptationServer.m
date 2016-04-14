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
global joints_imit;
global joints_adapt;
global joints_improved;
global y_imit;
global y_adapt;
global y_improved;
global target;

pool = gcp;
weights = [];
contexts = [];
config = [];
joints_imit = [];
joints_adapt = [];
joints_improved = [];
y_imit = [];
y_adapt = [];
y_improved = [];
target = [];

%% Configuration
config.horizon = 11;
config.step = 11;
config.umax = 2;
config.dmin = 0.01;
config.scale = 1;
config.alpha = 1;
config.dboarder = 0.01;

%% Contexts initialization
contexts.boarder = [0.8 -0.2];
contexts.ground = -0.2;