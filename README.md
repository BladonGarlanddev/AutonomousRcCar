# AutonomousRcCar
## Background Information
This project was part of an robotics competition I found through my engineering class in highschool. The professor reccomended this compeitition because he believed it would challenge me, and oh boy, he was right. A total of 600 hours went into this project, 500 from me, 100 from my 3D modeling partner. On average I spent 10 hour per day, including weekends, on this project which is staggering when you consider it was a 3 hour class. In two months time, an autonomous car capable of navigating complex, 3 dimensional spaces, was built from nothing.  

## Performance At Competitions

### National Robotics Competition 
The car ran for the first time the day before the actual competition. I didn't sleep for two days straight to ensure that it ran. One of two possible disastrous events occured: either a hardware malfunction after shorting a large battery or software issue caused by a sleep deprived zombie. Regardless, the IMU was reading degree values of over 900 when it should stay within the bounds of 360. This lack of accuracy caused the car to steer as if controlled by a 'bang bang' controller versus the PID controller it actually possesed. That, and the fact the obstacle course is unchanging led to the car not placing at the event. 

### Digital Electronics 
With a beautiful stand displaying the intricacy of the code, thoughtful design of the cars body, and impressive array of electronics, the car easily won me and the team the digital electronics competition.

## Design Concepts
The way the car knew where to drive was actually manual. I would walk around and plot coordinates where I wanted my car to drive by pressing a button on it. The car would then sequentially drive to those coordinates. 

The car possessed an after market ESC which allowed me to control it's drive motors with an Arduino Mega. The car used a GNSS and an IMU to navigate. The cars 3D printed body offered space to hold the various components while also being stylish. The windshield of the car was an LCD screen for diagnostics purposes. 

## How To Build Your Own

1. Understand your time limit and build options
   - If you want to build a quick autonomous car, a good IMU, GNSS, and a custom printed circuit board are your best friends.
   - If you want to build something a little more advanced but time costly, go for a 360 LIDAR and IMU.
   - If you want to prove you're smarter than Elon, try a couple of cameras, LIDAR, IMU, GNSS, and Artifical Intelligence. That'll show those lazies at Tesla.

2. Program it
   - Since I only programmed the GNSS and IMU type, I will only speak on it. 
   - The first thing you need is the formula to determine the angle between your car and the GPS Coordinate it would like to drive to. In my code, you will find it under the GPS class, its a function called getHeading(). Here is the formula: atan2(detla_lat, delta_lon). 
   - The second thing you need is the distance formula. In my code I used the Haversine formula which is largely unnessecary considering the ways in which the car will be used.
   - Lastly, you'll need to understand the concept of PID controllers. P is proportional. I is integral. D is derivative. Each of these deserve thier own paragraph.



## What is a PID controller?

A PID controller is a concept where the 3 factors are used in an equation that determines the way in which an object moves. For example, if you wanted your car to steer towards a gps coordinate, you would want that car to turn sharply towards its destination whenever its angle if far from its target. Whenever the car is nearly facing it's desired destination, you shouldn't turn sharply to correct your angle. PID controllers create rules and add behaviors to how your car will maneuver.

### Error:
To understand the other terms, we must understand error. Error is a measure of how far you are from a desired target. If a car was facing 0 degrees relative to true north and it's destination was 90 degrees, your error would be 90. Error in this context is a signed integer where values can be either negative or positive. Negative error is not better than positive. Any amount of error other than 0 is bad.

### Proportional:
Proportional is the first, easiest to understand, and most common factor in steering controllers. Your proportional term is proportional to your error. To visualize, if your car's error was really high, meaning it's far from facing it's destination, you would want it to turn sharply towards it's destination. Conversely, you'd want it to hardly correct its steering angle if close to what it should be. That is the P term. 

### Derivative:
The issue you will find with the proportional term is it doesn't account for angular velocity. That's a scary phrase but all I'm saying is the P term doesn't account for left over rotational momentum. Once your car's P term gets the car to 0 error, the car may still have momentum from turning towards its distination. That will cause the car to overshoot which leads to oscilacation in the system. The derivative term solves this effect by dampening the cars turning rate whenever the car is quickly approaching it's target when the error is relatively low. 

### Integral:
The integral is integral in making your car run. Lets say the car thinks it's driving towards its destination but a minor issue like loose steering links or a loose IMU is causing your car to drive 120 degrees relative to north when it should be driving 130. To counteract an effect like that, you can use the integral term. The way I used the integral term was an accumulation of error within a 20 integer long array. The array acted as a buffer where a each reading of error would be shifted linearly through each position in the array until it was discarded. The average of every value in the buffer was the integral term I used. I intentionally designed the integral in this way because it's a strong indicator of the accumulation of error over time while also not creating huge reactions to small amounts of error. The buffer also made sure the data was relevant at the time it was being used since new values are added and removed every 250ms. 

## Predictive Modeling

The model of autonomous car that has been described is largely depdent on it's GNSS. If you have unstable GNSS signal, your car may behave iraddically. It's not easy to diagnose a car when it behaves iradically. One way of combatting this is to use predictive algorithms for the cars latitude and longitude during an extended period of time without GNSS response. The algorithm I developed basically took the cars rate of turning, it's error, and it's distance to a GPS coordinate and then modeled the arc the car would drive if it wanted to get to its destination and fed that back into the PID controller. If I could explain it clearer, the car basically would take the data available to it at its most recent GNSS reading, make a guess where it would be in the next 250ms, then convert that into a lat and lon and feed it back to the PID cotroller as if it were lat and lon from a GPS

## Physics Simulators
When working with programming and hardware, the difficulty of combining the two are not added, they're mutliplied. It becomes almost impossible to diagnose issues visually after a point. It's hard to pull diagnostics information that can be linked to the cars behavior in a physical environment. At some point you will need a simulator like Gazebo physics to actually understand and fix issues. 




