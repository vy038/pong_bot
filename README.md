# PONG BOT V1.5

Manual Control Demo <br>
![Pong_BotManualDemo-ezgif com-optimize](https://github.com/user-attachments/assets/dd1ae97f-c8b9-466b-a60a-9e3eb2252fb0)

Auto Control Demo <br>
![Pong_BotAutoDemo-ezgif com-optimize](https://github.com/user-attachments/assets/39698ffa-3c8b-4bfe-8cef-138e1dcfbbaf)


An advanced and ambitious project I wanted to make with some spare parts and kits in my home, where a robot would be placed on a ping pong table with a bunch of ping pong balls inside. A gun with a firing mechanism I designed would lay on top of a chassis with omnidirectional wheels and IR distance sensors on the back and sides. The robot would be completely autonomous, using a random number generator to decide where to move (uses sensors to ensure it doesn’t fall off the table), and when to fire a ball. The gun was designed to hold multiple balls, allowing me to practice playing ping pong alone. It also has an ESP32 devkit attached to wirelessly receive signals from a wireless controller, allowing it to switch from manual to autonomous.

# CHANGE LOG
V1:
There would be a DC motor (although I could not find a strong one, I will get that later for an upgrade), that would pull on a string attached to a firing pin, and once it pulls back a certain amount, the pin would lock into place and a ping pong ball will fall in front of it. There will be a servo just underneath where the pin locks in, and it would push down on it, unlocking the pin (which has powerful springs), and forcefully hitting the ball, making it fly out of the gun, and also locking the next ball in queue on top of the pin, so the process can repeat with the next ball. I had to learn a lot of new concepts to try and make this work, including having to use millis() instead of delay() so it would not interrupt anything, and also learned how to design mechanisms that would prove effective and learned about supersizing a lot of my projects, as this was by far one of my biggest, requiring a total of 5 DC motors (controlled with a motor shield consisting of 2 L293Ds, and the last motor being controlled by an external L293D), a servo, and 3 IR proximity sensors. 

Some improvements that can currently be made include some improvements to the power supply, as the power currently being supplied to the bot is nowhere near enough to power the motors, and also getting a stronger motor/different mechanism to pull the pin, as a DC gear motor is not strong enough (use servo motor). This will be improved upon in the second version. 

UPDATE 1.2 (2024-04-13):
The firing mechanism now works perfectly, due to the swapping out of a weak DC hobby motor for a continuous servo, and moving the location from the front of the gun to the back, because I noticed throughout a lot of my trials, whenever I went to cock the pin back, I always did it from the top because it was much easier instead of where the motor was pulling, so moving it helped alleviate a lot of the stress on the motor and allowed it to fully pull the pin back. I also created a 3D design of a pulley/wheel attachment for the servo and got it 3D printed to be used for the servo. However, the printer was not precise enough to print out the teeth of the wheel to mount on the servo, so I had to super glue an existing servo head on top to create a reliable mount. Also, removing the DC motor helps bring the power requirement down a lot, and it now doesn’t require the extra L293D chip that I had originally.

Minor Update (2024-04-14):
Added 3 18650 Li-ion batteries to try and power it up, putting two in parallel and one in series to another, but the motors would still not run. I also added an extension to the ball holder so now it can hold up to 4 balls. I plan on taking a break / doing more research into why the motors are not working (could still be that there is not enough current). Motors work fine independently, but when run with other motors at the same time, only one of them runs. The current of 1 motor measured without load is ~200mA, and with load ~550mA.

Minor Update (2024-04-15):
Managed to get the motor movement working at the same time as the servo firing mechanism, the only thing left to do is change the motor speed for each one (some are faster than others, which can easily be done with trial and error), get the IR distance sensors working, tune some values for servo firing mech (optional), and add more batteries to the battery pack (optional). Turns out the problem was not the power supply, but an issue with my code. The IR distance sensors were later fixed so it could successfully detect edges and move the bot accordingly, however, the bot's movement is rather erratic, and it keeps moving in a circle, probably due to some motors not getting enough power, which could be a motor problem or the battery is not big enough. I will try to expand the battery size later and add heatsinks for the chips after.

Minor Update (2024-04-18):
Added one more battery in parallel to the lone battery as an attempt to get the current up, still to no avail. After some more experimentation and further research, the problem seems to be the motor driver (driver bought from China, but based on the Adafruit Motor Shield V1), which many people online say is a “worthless piece of junk” and should be “chucked as far as possible.” This is mainly because the chips (L293Ds) are considered ancient, and even Adafruit discontinued it in exchange for the Adafruit Motor Shield V2, so, if possible, I will try and research a better alternative to the current moro shield, one that is hopefully able to distribute current more evenly amongst motors. Although it could just be bad quality on the manufacturer’s part, as I got it for ~$1 from China, so I will try to test with a higher quality shield too.

Minor Update (2024-05-25):
Got a new motor driver shield (same model), but there was no difference when I swapped them out, the same issue was still present, which could mean two things: the model of the motor driver is bad, or it is the fault of the motors (unlikely, because I tested the motors and they seemed fine). Also attached a gyroscope to the robot, and made it adjust for the minor turns, but that didn’t end up working either, might polish up the gyroscope code later. Can also try to use another motor driver I have at home that can only give commands to two motors, but can make it work if the wheels are given commands in an ‘X’ shape, still allowing it to move all 4 directions but only controlling two motors at a time.

Update (2024-08-22):
Replaced the motor driver shield again, but this time for the Adafruit motor driver shield V2, which is way more energy efficient. This also shows in the final result, as the differences in motor voltage dropped drastically, and really only shows when the bot has to travel long distances, which it won’t have to do anyways. The real problem with the motors moving inconsistently however, was the uneven weight distribution, as a lot of the heavy batteries were at the back of the car, so moving them to the front solved the issue. However, another problem arose where the servo that pulls the trigger back is not strong enough. I then replaced the SG90 with a stronger MG995, and also cut the trace on the motor shield V2 to connect an ext. Power supply to the MG995. I had to also solder and make a modification to the shield which allowed for the other servo to remain on regulated 5V power if needed. The only things left to do now are to 3D print a bigger wheel for the MG995 to pull the trigger (3D model done, waiting on print), and also to wait for a four AA battery holder to supply 6V external power to the servo(s) with good current. Once that is done and no further problems arise, the project will be finished.

Change (2024-08-27):
Not really a change, but added a different mode to the robot. Attached an ESP32 to the robot to communicate with the Arduino for wireless control. Hooked up the circuit from ESP games (ESP galga circuitry) and turned it into a controller for the robot, controlling where it moves and shoots. Aside from that, no major changes besides adding a US sensor to the front for net avoidance.

Big Update (2024-09-08):
UPDATE 1.5: Added dual modes for the robot, allowing it to switch from auto to manual. Also managed to find a $1 fishing line reel that is the right size for the MG995 motor, and was able to successfully pull back the firing pin with no effort. I also added a new support to the motor, securing it more soundly to the top edge of the gun, preventing a lot of the bending of the gun as the trigger is pulled. I also switched a lot of the power supplies around; hooked the two li-ion batteries to a 5V regulator on the breadboard (had to also rewire some of the stuff on there), using it to directly power the sensors and common grounding it with the Arduino. Using regulated li-ion batteries ensured that the servos had enough current they needed, but also didn’t get overloaded with voltage, operating at a steady 5V. I then used a simple 9V battery to power just the Arduino for the logic, utilizing the Vin pin.

Small Changes (2024-10-01)
Added an edge sensor to the front of the bot in case it decides to turn around and run over the edge of the table (it did that and broke before), and if possible, secure gun more firmly to chassis to prevent it from wiggling as bot moves around. Aside from that, I 3D printed a cover for the game console that improves the look and appeal of the bot to others. 
