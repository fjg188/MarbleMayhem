# Project Descriptions for Team fjg45-rlw332

Finalized Project Proposal: rlw332, fjg45. We will create a maze solving game where the board will act like a controller. Using a python GUI with procederally generated mazes, so that every time someone plays it is a different maze, the player will use the MCUX board to control a marble placed within/at the starting point of the maze. They will control the marble by tilting the board and using the onboard accelerometer, We will use the board to detect the orientation of the board relative to the ground/standing position based on the ways that the board is tilted via the user. This processesing of the sensor data will be done locally on the board as well as making the sensor readings compatable with the communication protocals we will use to send the data to the computer thats running the GUI. 

## fjg45
We want to use the gyroscope in the board as a controller and tilt it to make a marble go through a maze. 
## rlw332
Refer to Felix’s response for information about the high-level design for the project. 
# Feedback

Ok. Sound like fun! It might be easier to use the accelerometer to detect the direction of gravity (and thus the tilt) as opposed to measureing the angular speed, which is what the gyro would measure. 

Also, keep in mind since this is a class about embedded programming, make sure that the raw sensor readings are interpreted on the FRDM board, as opposed to sending raw sensor readings back to the host computer and interpreting them there. The latter would not be sufficiently complex as a project. 

# Project Web-Page

The project web-pages will also be hosted on github in this repo in the "page" branch. You can edit it by switching branches and modifying the files, or by pushing to the branch. Here is a link to a minimal web-page that you can edit and modify: [https://pages.github.coecis.cornell.edu/ece3140-sp2025/fjg45-rlw332](https://pages.github.coecis.cornell.edu/ece3140-sp2025/fjg45-rlw332)
