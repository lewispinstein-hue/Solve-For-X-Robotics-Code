LOG

For the people is robotics class:
##### I have put the main code in a toggle loop. Push up arrow + left arrow at the same time to toggle between driving mode (and checks) to a debug mode. #####

use Shift+cmd+p and run PROS: Tntegrated Terminal to open a PROS terminal

Biel gmail: bielgiliharik@gmail.com
Lewis gmail: lewispinstein@gmail.com

spreadsheet for controls:
https://docs.google.com/spreadsheets/d/15B_y9Lt4vU23R9hhuVtT6RbHhSsFFPGtf5XDaz9-9wo/edit?gid=0#gid=0

google doc for comments and other:
https://docs.google.com/document/d/1omgR9gJ0i7vaN7-qcAjP3cJlbeNEO6gLahb0yOsdxQU/edit?tab=t.0

We are using arcade double stick with a 0.25 bias for the drivetrain

functionality:
Left joystick = forward + backward
Right joystick = left and right 
R1 = spins for middle goal
R2 = spin for upper goal
L1 = 
L2 = (we removed de-scorer pneumatic)
x = 
B = toggle intake 

ports: 
left dt = -3,-14,13
right dt = 8, -20,18
1st intake = 2
last stage = 9 

scoring arm = TBD

arcade double stick
0.25 desaturatebias
