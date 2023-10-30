# AutonomousRcCar

BACKGROUND INFORMATION:
This project was one that came out of no where when my teacher and I saw the autonomous car challenge from the NRC. It was two months before the competition would begin and I had not a single idea how to design or program an autonomous car. Despite this, I managed to get a basic understanding of how they are built, and started building. By the end I had 600 hours in this project. I controlled every aspect of the project from begining to end including, the choice of using an RC car, the style of RC car, the MCU, the mounting for sensors/IMU/battery-pack, PCB + design, LCD, and the sensors. Nobody in my class had ever built anything like this so I was basically left to figure it all out on my own. Through trial and error, I realised I didn't have time to handle using a ton of sensors so I opted for a high-quality U-blox GNSS and adafruit IMU (which was also, ofcourse, after using the wrong sensors for a while). Though the programming went through many, many, many, iterations I finally found a certain structure that worked for me and a certain train of thought that helped the car progress in a more linear fashion. The beginings of the car actually started with super sonic sensors and lidar until I realised I didn't have time to mess with something so complex such as kalman filtering. After that, I got the first magnetometer script to work and make the car drive towards north. It was really a surreal moment in a sense. After that, I began tryng to incorperate GPS sensors into it but after mishaps of failing sensors, inaccurate sensors, and a lack of libraries for the sensors, I was delayed greatly. Infact, I had around 1000 lines of code responsible for handling GPS related functions until I got ahold of a Ublox GNSS which came with many built in functions. Before that, I was quite literally parsing like raw GPS signals and if you look at them you might get an idea of why thats annoying. While I made large strides while using a Ublox GNSS, the magnetometer was holding me back significantly so I had to look for something more accurate which is where I found adafruit's IMU. From there progress was faster than ever until I started hitting roadblocks where it wasnt feasible to test the car in a physical environment. I needed a physics engine to simulate the car because the program was getting too complex to understand visually and I needed a better way to diagnose problems with the car. At this time school was in its last week or two and the competitions were over so I opted to work on the hardware instead. 

How to use? 
The car simply had a button I could press that would save a GPS coordinate into an array and I would basically press it as many times as I needed until I had enough points that I felt confident it would navigate the course as I desired. Once I had the coords, I simply set it down, pressed a button, and let it drive.

National Robotics Competition Results:
The competition went horrible. I accidentally shorted a wire and it killed a 200$ GNSS so I was forced to basically completely rewire the car on the spot and do diagnostics hours before the competition. What I failed to realise was the IMU was also destroyed during the short and was reading values of like 900  degrees when its supposed to be limited to 0-360 so yeah. Ironically it almost finished the course despite this but it acted a lot more like a bang-bang controller than a PID controller. Also, I was the singular individual to have a even semi-autonomous car. Everyone else's cars were hard-coded and thus could be programmed to do the course as efficiently as possible rather than being flexible to any course. I knew that the competitors wouldn't have genuinley autonomous cars because no typical highschooler would put in the 600 hours it takes to get as far as I did and not only that but the course was unchanging so there was no incentives to make a genuinely autonomous car. 

Code explanation: 
There are a few fundamental scripts in this code that are vital to building an autonomous car in the way that I did. 

One of the first but most simple scripts was finding the shortest path around a circle. You cannot simply tell a car, if you want it to turn towards an direction, to turn right if your degrees are less than the desired degrees or vice verca because that would ignore the situation where you are at degree 1 and the target is at 359 degrees. You have to check across the zero as I call it. And you have to do it both ways. I dont know if there was a more efficient algorithm for this than what I made but it atleast worked.

Another fundamental script was figuring out what degree you would need to face to be facing towards a GPS coordinate. This one gets really confusing so I'd just follow my code to see how I figured out what degree I need to face in order to drive towards a coordinate

Finding your distance to an object using GPS coords is also pretty important thing because it helps with knowing when youve got to your coordinate as well as helps with your speed controller. I thought it was fairly simple.

PID controllers are so important its not even funny. You can have everything else in your program right but if you dont have a solid PID controller your car will look like its being driven by someone who downed a bottle of jack. In simple terms, your P term stands for proportional and its proportional to the error or in my case, its proportional to how far off my car is from a desired degree. For example, if my car is supposed to face 90 degrees but im facing 160, i have an error of 70. In my case, my proportional function was actually a sigmoid graphically and I did that to hit the sweet spots where the car should turn hard or turn softly.

The derivative term is also incredibly important. The derivative term is basically how fast your error is changing or accelerating as you'll commonly hear. In more complex terms, its the angular velocity of an Autonomous car. Basically, the derivative term asks the question "how fast are you approaching your target value" Which in my case would again be a certain degree. This example may help you understand. Sometimes, you will notice your car is turning very hard towards a desired target and realise the proportional term doesnt account for the force of the car turning and so it will overshoot its target often leading to oscilation. I realised intuitvely before I knew what a PID was that the issue with my car was that it would still have angular velocity remaining when it reached its target and it would overshoot past its target. Accounting for angular velocity allows you to slow the car down even more when it is approaching its target too quickly. It is very, very useful. 

The integral term was not neccesary in my specific case. EDIT: now that I look back on it, the steering in my car wasn't perfect because I had to modify the steering axle, so the integral term actually was correcting for my imperfect steering by noticing the car wasnt driving towards the desired angle. My cars derivative term dampened the steering speed excessively. Sometimes you simply want the car to turn sharply but also not have it overshoot and for me, the integral term helped with that. The problem with the derivative term is if your error and angular velocity are large, the car would display a larger dampening affect. Any time your error is large, you want the car to turn sharply. What the integral did was it would counteract the derivative by forcing the car to turn faster if the error remained high over a period of time. With the forgetful style integral, the intergral would stop affecting the steering greatly as the cars error dropped. Basically, I had an array of like 20 numbers and those numbers were an accumulation of my error over time. Overtime, old integral values would be rotated out and replaced with newer data. Doing this meant that my integral would only have a significant effect if the error remained high over a period of time. 

Side note: I have been told by reddit that using a complex derivative like how my proportional term is a sigmoid is a bad idea but I don't believe that. I likely could have avoided using an integral term if I made an equation that takes into account the pure error in the derivative term.

Accounting for bad GNSS signals became important. There was a minimum delay of 250ms between each reading and some readings would be inaccurate or delayed and so it became apparent that I had to find a way to solve it without relying on better hardware (my GNSS was top tier). I didn't really find anything simple that I could follow along with on the internet so I kind of figured that at any given point the car is traveling at a certain speed with a certain degree of semi-constant rotation over a interval of time. So basically, I just wrote some code to predict the arch the car would drive, the convert that change in x and y into changes in lat and lon and fed that into the steering controller. While mathematically, it made some sense, the real world was not so forgiving and all the different factors in this car made it virtually impossible to diagnose any problems with the algoirithm through visual observation due to its growing complexity. I would very strongely advise you learn linux systems and download Gazebo physics to simulate your car. I unfortunately did not have knowledge of linux systems at that time and thus did not successfully use the simulator. Without the simulator I was left guessing if it was a bad GPS reading, IMU reading, sub-par PID controller, or faulty prediction code. 
