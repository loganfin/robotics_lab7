# Lab 7

Beaker: Robot A

Bunsen: Robot B

## Sequence of Events

1. Robot A: picks up the die

1. Robot A: drops the die at the first conveyor

1. Robot A: turn on the conveyor for a few seconds

   - Do both conveyor belts run at the same speed?
   - Only need to calculate the X coordinate
   - Create a function that takes a float for time and returns a shifted X
     coordinate.
   - Find the velocity

1. Robot A: send a message with a ROS topic

1. Robot B: pick up the die, rotate about the X axis, rotate about the Z axis

1. Robot B: place the die on conveyor belt B

1. Etc.

## Misc Notes

- We can have multiple threads, just a single node

- We may have to busy wait for the other topic to be created before we create a
  listener for it

- Turning on/off the conveyor belts could be a service (or services)

- Moving the robot arms could be a topic

## Logan

- Figure out the time/distance function

## Ben

## Topics

- "/logan/time"
- "/ben/time"
