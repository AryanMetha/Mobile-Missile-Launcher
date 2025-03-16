<h3 align="center">For Motion Planning</h3>
<h2>PID Control for autonomous path following</h2>
PIDv1 - basic code with error wrt angle subtended to the next waypoint <br/>
PIDv2 - fixed many logical errors and implement rviz and matplotlib visualisation<br/>
PIDv3 - entire revamp of the code, error is now the cross track error.<br/>
PIDv4 - pid paramaters now support auto-tuning with respect to any path.<br/>
PIDv5 & v6 (LATEST) - pid enhances(not directly runs) the motion of the car. fixed this core issue. more realistic turning mechanism with angular velocity support <br/>

<br/>
<h4>PIDv6 (LATEST) visualised</h4>
![image](https://github.com/user-attachments/assets/915a8429-336a-4b62-9216-0037165c3326)
<br/>

<h5>The car is no longer blind (as in previous codes). The car drives itself while pid control fixes the crosstrack error (which should have been from the start but implemented finally in v6.)</h5>

![image](https://github.com/user-attachments/assets/bfd347b3-d0b1-48e6-905c-ed4621094299)
<br/>
<h4>Image showing the auto-tuning process for pid params (pidv4)</h4>
![image](https://github.com/user-attachments/assets/d7525bf5-060b-4899-85c1-1fc37aade531)
<br/>

![image](https://github.com/user-attachments/assets/97ba3cbd-a407-412f-94dd-6d47729676f0)
<br/>
<h4>Image showing the path followed with the tuned parameters for a sinosuidal curve (pidv4)</h4>
<br/>

![image](https://github.com/user-attachments/assets/a6b80ed3-f293-4f32-b2f0-0466cbb2f9ef)

<br/>
![image](https://github.com/user-attachments/assets/0c5d7435-6a3d-43f3-a70c-1a3cbf9a4506)
<br/>
<h4>Gazebo simulation for the car. The basic differential drive model is ready. testing is yet to be done</h4>
<br/>



<br/>
Also started changelog tracking in this repo
<br/>



<h2>Initial works</h2>
Generates an Arbritary Occupancy Matrix 
Calculates target and its points
Generates a straight line path to reach at a certain distance before it.
<br/>
![image](https://github.com/user-attachments/assets/fb11d7fc-932d-4b48-b570-4bedd5ed120a)
<br/>
Plots a straight line path as shown. 
Magenta dot is start position.
Green dot is target/stopping postion.

![image](https://github.com/user-attachments/assets/a2c3da60-ce9e-42d3-ba4d-ffea3b64b30e)
