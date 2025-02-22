# GA-optimized-MLP-based-LPV_MPC
This is the implementation of the work published in the following paper "Autonomous driving using GA-optimized neural network based adaptive LPV-MPC controller". 
The paper is freely accessible at this link: https://upcommons.upc.edu/bitstream/handle/2117/383650/Autonomous_driving_using_GA-optimized_neural_network_based_adaptive_LPV-MPC_controller.pdf?sequence=1 

## Steps to run the code:

This implementation requires MATLAB, Yalmip, and at least one solver such as gurobi, quadprog...etc.
you can install yalmip by following the steps in this link : https://yalmip.github.io/tutorial/installation/ 
To run the Python code and build a neural network and train it you need to install the libraries used in the code :

![image](https://github.com/yassinekebbati/GA-optimized-MLP-based-LPV_MPC/assets/49916054/7566d0c9-30a0-490a-98ff-06b768ea159c)


-  # LPVMPC:

   1. You can run the LPV_MPC controller by launching the script "MAIN_LPV_MPC.m".
   2. The simulation requires data for trajectory, disturbances, MLP network...etc, these are loaded automatically by the script (make sure all files are in the same folder).
   3. Try and experiment with different parameters for the controller (Prediction horizon Np, constraints, Q and R weights), you can also test the controller in different trajectories (two are provided: double lane change trajectory in "DLC_pos_data.mat/DLCwind_data.mat/DLC_data.mat" and a general trajectory in "Pos_data.mat/test_data.mat/wind_data.mat").
          

-  # GA_LPV_MPC:

   1. To launch the GA optimization for the LPV_MPC controller, just run the script in the file (LPV_MPC_APP).
   2. The GA algorithm will optimize the MPC weighting matrix Q.
   3. Try optimizing with different hyperparameters for the GA algo, you can modify the number of iterations, number of populations, Min/Max limits for optimization variables as well as other parameters  (see "%% GA Parameters" in LPV_MPC_APP.m).
   4. You can evaluate the GA algorithm on a simple sphere function that runs relatively fast by running the script (SPHERE_APP.m). Note that the GA algorithms need these files to run (Mutate.m/RouetteWheelSelection.m/RunGA.m/SortPopulation.m/TournamentSelection.m/UniformCrossover.m)

- # Neural_Network:

   1. Run the Python code in (MLP_LPV_MPC.py). This will build a neural network and train it using the data in the included xlsx file.
   2. Run the script (Network_Import.m) to convert the built and trained Keras network to mat data. you can use it in the LPV_MPC controller.
   3. You build different networks by changing the structure (depth/width) of the neural network and modifying the hyperparameters: learning rate, batch size, epoch..etc.

## If you find this work useful or use it in your work please cite the main paper:
#### APA:
Kebbati, Y., Ait-Oufroukh, N., Puig, V., Ichalal, D., & Vigneron, V. (2022, December). Autonomous driving using GA-optimized neural network based adaptive LPV-MPC controller. In 2022 ieee international conference on networking, sensing and control (icnsc) (pp. 1-6). IEEE.

#### BibTeX:
@inproceedings{kebbati2022autonomous,
  title={Autonomous driving using GA-optimized neural network based adaptive LPV-MPC controller},
  author={Kebbati, Yassine and Ait-Oufroukh, Naima and Puig, Vicenc and Ichalal, Dalil and Vigneron, Vincent},
  booktitle={2022 ieee international conference on networking, sensing and control (icnsc)},
  pages={1--6},
  year={2022},
  organization={IEEE}
}
