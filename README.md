# ACC

Astar file contains all the files for A* algo, run astar_main to run the planning algo for the whole track. 
The GPS data is the data from the webots simulator 
mpc algo is for MPC controller - genetic algo is to tune the weights of MPC based on the cost. 

# Need to do much better documentation 

First - Get the controller working properly into the webots 

Webots have a time delay issue of 3 time steps 
If the actual car also have the similar problem -- will work on this 

This implementation is taken from

[Advances in Industrial Control ] Liuping Wang (auth.) - Model Predictive Control System Design and Implementation Using MATLAB® (2009, Springer) [10.1007_978-1-84882-331-0]
LINK - https://www.researchgate.net/profile/Mohamed-Mourad-Lafifi/post/How_to_design_model_predictive_controller_in_a_factory/attachment/5d2dae0b3843b0b9825ae2b9/AS%3A781245130735617%401563274762980/download/Model+Predictive+Control+System+Design+and+Implementation+Using+MATLAB_Wang.pdf

The state space eqn is of the velocity form   and not position form
ie state space eqn looks like  eqn 1.5  in page 5 of book
Notice the states -  augmented matrix X  = [dx , x]T


Post this code gets working on the actual vehicle -- all the previous codes will be removed. 


# Nxt steps are 
Feedback from the car 
Integrating radar based distance and velocity 
Image classification to lock onto a target 
Synchtonising radar with image 

Putting all together into a single system 
Testing and tuning
