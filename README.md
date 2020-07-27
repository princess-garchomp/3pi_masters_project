# 3pi_masters_project
this is the final source code for the project required to finish my M.S. in electrical engineering

this was a bare mettal project, no outside drivers were used in the final implementation

link to 3pi 
https://www.pololu.com/product/975

the goal of this project was to have two 3pi robots communicate and solve a line-following maze
both would reach a maze topology for a stopping critera. based on the stopping criteria the robot would either:
belooking for a signal to move left or right 
or the robot would be signiling to the other to turn left or right

I used the available I/O. two remote control cars that I salvaged and some transistros to create the communication between the robots.
if the robot needed to signal "go left" it would set the corresponding output pin high, closing a transistor and making the remote control car circuit signal to the other robot
if the robot needed to detect the signlaing it would detect if a transistor on its input pin was closed by the receiver board from remote control car
