# Catching-a-Moving-Target

A planning algorithm for catching a moving target in a grid-world environment with obstacles.



## Compilation and Visualization

To compile the project, navigate to the project directory and run the following commands:

```bash
#compile
cd ～/zihanqu/code #Open a terminal and navigate to the code directory.
g++ src/runtest.cpp src/planner.cpp -o run_test

#run
./run_test map<number>.txt

#visualize
python3 scripts/visualizer.py maps/map<number>.txt
```

