# Nitrox Analyser Assembly

Make sure that you have all these parts available for asembly in addition to the 3D printed case:
![Needed parts](./images/1.jpg)

## Board Assembly
The first steps of the assembly are to assemble everything to the central board.
We'll start with the connector for the oxygen cell and the connection of the analog-digital converter.

### 1. First connectors
You need these parts:
![Needed parts](./images/2.jpg)

The connector for the oxygen cell goes in like this:
![Needed parts](./images/3a.jpg)
![Needed parts](./images/3b.jpg)

The connection for the analog-digital converter goes in like this:
![Needed parts](./images/4a.jpg)
![Needed parts](./images/4b.jpg)

After soldering them it should looke like this:
![Needed parts](./images/4c.jpg)
![Needed parts](./images/4d.jpg)

### 2 Connector for arduino
Next we'll add the connectors for the arduino. You need these parts for the step:
![Needed parts](./images/5a.jpg)

Put them into the board like this and make sure that the longer side of the connectors is on the board side:
![Needed parts](./images/5b.jpg)
![Needed parts](./images/5c.jpg)

Then solder them in and it should look like this:
![Needed parts](./images/5d.jpg)
![Needed parts](./images/5e.jpg)

### 3. Connector for display
Next step is the connector for the display. You'll need this part:
![Needed parts](./images/6a.jpg)

It does into the four hole named "screen" on the opposite side to the ocygen cell connector. Like this:
![Needed parts](./images/6b.jpg)

After soldering it should look like this:
![Needed parts](./images/6c.jpg)
![Needed parts](./images/6d.jpg)
![Needed parts](./images/6e.jpg)

### 4. The digital=analog converter board
Next part is the analog-digital converter board.
*Please note, that I cut off the pins underneath the board to make sure that there's no unplanned connection. Check your setup and if needed clip off the pins. If there's enough space you don't have to.*
![Needed parts](./images/7a.jpg)

Here I am using one of the wire pairs to make sure that I sorder it in parallel to the main board...
![Needed parts](./images/7b.jpg)

After soldering it should look like this:
![Needed parts](./images/7c.jpg)

Here you see why I cut the pins there...
![Needed parts](./images/7d.jpg)

### 5. The arduino nano
Next we'll add the arduino nano:
![Needed parts](./images/8a.jpg)
The arduino needs to be added like this. Check that the GND pin is next to the square hole and the D4 is next to the round hole:
![Needed parts](./images/8b.jpg)
![Needed parts](./images/8c.jpg)

And after soldering this should look like this:
![Needed parts](./images/8d.jpg)

### 6. Preparing the external parts
Next we'll solder the wires onto the button and switch:
![Needed parts](./images/9a.jpg)
Make sure to use the middle and the connector at the '0' of the power switch:
![Needed parts](./images/9b.jpg)

### 7. Connecting the button, switch and battery connector
You'll need these parts:
![Needed parts](./images/10a.jpg)

#### Note that you should do step 11a first if you don't want to do the soldering here twice, like I had to when writing the documentation...

The external cables are connected like this...
battery at "VC" with red wire soldered into the hole marked as "+"
![Needed parts](./images/10b.jpg)

The button soldered into the holes named "Button"
![Needed parts](./images/10c.jpg)

And the power switch in the holes named "S1"
![Needed parts](./images/10d.jpg)

After soldering it should look like this:
![Needed parts](./images/10e.jpg)
![Needed parts](./images/10f.jpg)

### 8. Adding the display
Plug in the display like this:
![Needed parts](./images/11a.jpg)
![Needed parts](./images/11b.jpg)
![Needed parts](./images/11c.jpg)
![Needed parts](./images/11d.jpg)

### 9. Flashing the arduino
TODO: Add Documenation on flashing or the link to that document here...

### 10. Trying it out
After flashing is done turn it one send see if it works:
![Needed parts](./images/12b.jpg)

### 11. Putting it into the case
You'll need your 3D printed case for this:
![Needed parts](./images/13.jpg)

#### 11a. Button and switch
Put the button and the switch in. The button needs to be screwed and the switch is pushed in.
![Needed parts](./images/14a.jpg)
Should look like this:
![Needed parts](./images/14b.jpg)

#### 11b. the display
Put in the displace onto the four pins in the lid:
![Needed parts](./images/15a.jpg)
![Needed parts](./images/15b.jpg)

#### 11c. Connect the cables to the board (again)
If you did the test without installing the button and switch in the case first you'll have to solder them to the board again...
![Needed parts](./images/16.jpg)

#### 11d. Connect the board to the display
Plug the board into the display connectors and onto the four pins like this:
![Needed parts](./images/17a.jpg)
![Needed parts](./images/17b.jpg)

#### 11e. Installing the oxygen cell
Push the oxygen cell through the hole in the case and screw on the flow restriction cap:
![Needed parts](./images/18a.jpg)
![Needed parts](./images/18b.jpg)

#### 11d. Tidy up the cables
To make closing the case easier I used a few cable ties to tidy up the cables like this:
![Needed parts](./images/19.jpg)

#### 11e. Battery
Connect the ocygen cell to the board and give it a quick test before you do the final step. You don't want it to not work after you closed it...
![Needed parts](./images/20.jpg)
Then put the battery into the battery compartment and close the case:
![Needed parts](./images/21.jpg)

### Congratulations!
You are now the proud owner of a fancy, new Oxygen analyser!
![Needed parts](./images/22.jpg)
