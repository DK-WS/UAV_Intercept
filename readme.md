#Introductions:
This is a project for quadrotors simulation.
#Documentations:
[Modeling and control of quadrotor](.\Modeling and control of quadrotor.pdf)  
[Introduction](.\Introduction.pdf)
#Packages:
Beside the packages that come with Python, this project need some extra packages:  
numpy: `pip install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple`  
matplotlib: `pip install matplotlib -i https://pypi.tuna.tsinghua.edu.cn/simple`  
#How to use:
* Set up the model's parameters in `.\model\quadrotors_parameters.py`   
* Set up the controller's parameters in the class ControllerParameters in `.\controllers\euler_controller.py`.  
* Run the `main.py`, the simulation is starting.The main.py is the main process of project.The instantiation of classes of the controller parameters, controller 
and models will be done in the main function. And cyclic call class method to complete the simulation. Finally, draw the 
curve with draw.py and draw3d.py.  
* The figures will be saved in the folder named images.
  





