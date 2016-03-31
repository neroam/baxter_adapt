import roslaunch
import rospy
import numpy as np
import cma

package = 'baxter_cut'
executable = 'cut.py'


launch = roslaunch.scriptapi.ROSLaunch()
launch.start()
#coefs = [-20e-4,20e-5,-10e-6,-100e-6]
coefs = [10e-4,10e-5,10e-6,100e-6]
final_result = []
# es = cma.CMAEvolutionStrategy([0,0,0,0], 4)
es = cma.CMAEvolutionStrategy([-3,0.9,-4,-2], 4)
# [-0.0003297, 9.12e-6, -3.9558e-5, -0.000209 ]


while not es.stop():
	evals = []
	solutions = es.ask()
	j = 0
	while j < len(solutions):
		print "Recorded data point: ", len(final_result)
		sol = solutions[j]
		
		theta = str(sol[0]*coefs[0]) + ' ' + str(sol[1]*coefs[1]) + ' ' + str(sol[2]*coefs[2]) + ' ' + str(sol[3]*coefs[3])
		print "Theta is :", theta
		node = roslaunch.core.Node(package, executable, args = theta, output = 'screen')
		process = launch.launch(node)
		while process.is_alive():
			pass
		res = np.load('/home/aecins/output.npy')

		check = raw_input("Do you accept this result? ")
		if len(check) == 0:
		   final_result.append(res)
		   evals.append(-res[1])
		   np.save('/home/aecins/final_output', final_result)
		   print "accept result"
		else:
		   print "reject result"
		   j -= 1
		j += 1
		process.stop()
	print "Evals is:", evals
	es.tell(solutions, evals)
es.result_pretty()
