<h3 align="center">For Motion Planning</h3>
<h2>PIDv4</h2>
PIDv1 - basic code with error wrt angle subtended to the next waypoint <br/>

PIDv2 - fixed many logical errors and implement rviz and matplotlib visualisation<br/>

PIDv3 - entire revamp of the code, error is now the cross track error.<br/>

PIDv4 (LATEST) - pid paramaters now support auto-tuning with respect to any path.<br/>

<br/>
<h4>Image showing the auto-tuning process</h4>

![image](https://github.com/user-attachments/assets/d7525bf5-060b-4899-85c1-1fc37aade531)
<br/>

![image](https://github.com/user-attachments/assets/97ba3cbd-a407-412f-94dd-6d47729676f0)
<br/>
<h4>Image showing the path followed with the tuned parameters for a sinosuidal curve</h4>
<br/>

![image](https://github.com/user-attachments/assets/a6b80ed3-f293-4f32-b2f0-0466cbb2f9ef)
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
