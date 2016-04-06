clear all

imitation_name = '/imitation_server';
rosinit;
imitation_node = robotics.ros.Node(imitation_name);
imitation_server = robotics.ros.ServiceServer(imitation_node,['/baxter_adapt', imitation_name], 'baxter_adapt/Imitation');
imitation_server.NewRequestFcn = @imitationCallback;

