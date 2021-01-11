# Doomfist_Glove
A custom controller for the PC game Overwatch for character Doomfist.

Controller is based on arduino nano with MPU6050 sensor and a button. All of them are glued to a glove.  
Sensor data is being sent to PC, where it is processed in a python script, which then sends an input to the game.

## Features 2 modes:  
### Template calibration   
Allows to calibrate up to 5 gestures.  
Pre-calibrated gestures right now are:  
  
**CHARGE** - backwards hand movement  
**PUNCH** - forwards hand movement  
**SLAM** - downwards hand movement  
**UPPERCUT** - upwards hand movement  
**SLAP** - well, a "slap" movement  

### GameTime  
Real time gesture recognition after a button press on the glove.  
Totally capable of real-time gameplay.
