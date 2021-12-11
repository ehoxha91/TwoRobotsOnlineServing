# K - Robots (2) Online Serving Problem using Dijkstra and Heuristics

- Compile line:  gcc -o outputfile TwoRobots_001.c -lX11 -lm -L/usr/X11R6/lib
- To run the code: `./a.out input.in

![All_Text](https://github.com/ehoxha91/TwoRobotsOnlineServing/blob/master/OnlineTwoRobotsProblem.png)

This project is a solution of two-robot online optimization problem(k-server online problem).
I have some obstacles, that I give the program as an input file of a certain format (xxx,yyy)-(xxx,yyy). These segments form obstacles of the map. Robots will always avoid them. 

When we click with the left mouse button program will receive it as a 
request input and one of the robots will move. The algorithm in this project is not greedy algorithm. It uses the model of a simple neuron.
Based on some weights robots will act differently. The main goal was to keep the traveled distance difference of two robots as small as possible.
After few tests and calibrations of the weights by intuition I think that I acheived some good results, using some heuristics. 

Robots travels on the shortest path possible, from the current position to the request point (Dijkstra).

Detailed description of the problem is included on file "Project 2.pdf".
