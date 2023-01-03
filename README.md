# Rover-Project
Project: Search and Sample Return
Instructions on how to setup dependencies and the simulator can be found here.
A comprehensive discussion around implementation decisions can be found on this Medium post.
Results

alt text

The Rover does a good job following the left wall avoiding trouble by getting unstuck if necessary. It also collects most of the sample rocks it finds on its path. It maps up to 97% of the terrain with a fidelity of 73.5%.

Bellow is a 8x video of the Rover from start to finish:

Improvements and Future Work

I'm really happy with my submission but there are a few things I wish I had the time to tackle:

Return to the starting point

The final piece of the puzzle was to have the Rover to go back to the initial position and come to a complete stop. Unfortunately I'm already a week past the suggested deadline so I'll have to skip this one. There are many ways to implement this but the easiest (and for what I would go first) would be to save the initial position when the simulation starts and query if the Rover is close to this position AND all (or most) of the rock samples have been already collected. This condition would bring the Rover to a new state: "home" Once in the home state the Rover would, just like for the rock state, slow down and steer on the direction of the initial position, stoping when close enough.

Collect all the sample rocks

One of the problems that I notice with my state machine is that it goes back to a previous state whenever a rock is no longer in sight. This is problematic for rocks that are hidden under tight spots. When the Rover approaches and breaks violently that causes the camera to miss the rock for a few frames causing the state to go back to forward mode skipping the rock completely. To solve this would require improvements on both the state tracking and also the control of the Rover to avoid slamming the breaks when approaching a rock.

Use a PID controller to avoid oscillating behavior

Lastly, I would like to use a PID controller to make the Rover steering feel smoother and more natural. I’ve checked and PID controller is going to be covered in future lessons.
