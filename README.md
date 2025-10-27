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




For Next tournament:
Instead of directly applying the joystick to the motor, we can translate it into
a percent and then apply that percent to the motors. This would solve the
problems where, when turning, only one wheel moves.
For example: Y = 30, X = 30
the left motor would get 60 (y + x), but the right motor would get 0 (y - x)
making the drivetrain speed proportional instead of a direct calculation from
the joysticks would make the robot easier to drive

Problems for later:
The intake chain keeps slipping.
We need to add some gears on both sides of the chain that are bigger and stop
the chain from being able to slip

Auton mode:
We start the game with a ball, so the first priorty is to be able to score that
ball for the bonus +10.
After that, we should just make the robot pick up the three balls by the center goal and then score them

Add functions with global scopes that take in an input, and do the appropriate thing (e.g. setPnState(True/false), or )

Turning is very hard. We need to add some special math that makes turning be slower 
Maybe make turning and driving f/r seperate properites, so that we can make them customizeable


Guardrails
More rubber bands to the intake conveyor
More rubber bands to secure the motors.
Better descorer

Add a pneumatic that can stop balls from comming out of the top goal so we do not have to worry about accidently dropping balls

sawasdawddasasdawawda0sdawda-sd-a-wd-q2-1-gh-ehdk-aswdasddasvcawdaddawdsdaasdaccsdawdt-cLbv6arIa5wdasK0PSS8RHDPasdawdasdwjLTbPriC50cy3asdaqvTh36moEmtasdawhLKYONmXVUuawddasdSi7IBM3tpeWxulISZY8T3BlbkFJjQn8asdavdjCqyGYB3pL3awdawddasdR-PRt2zNgEv07Q6eSawdaYptb8C2nk1dhZHeaaOP5Z8whq1wdasd-awd-as-da-wd-adbMu25fJ26zfnQAawd