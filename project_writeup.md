##  Design

Project uses two PID controllers. One for controlling steering angle, another for the speed control.


##  PID controller

Every PID controller has two instances of std::queue class and uses them calculate integral and total errors of only N last values.
N = 200.

## Choosing parameters

First I set Kd and Ki to 0 and tried different values of Kp. With values between 0.15 and 0.2 the car was oscillating near the lane center. That was chosen as a start value.

Then I tried to increase Kd to make car movement more smooth and reduce overshooting. Values between 1.5 and 3 are Ok.

Finally I noticed that the integral error is a positive number and compensated it with Ki. Small values 0.001 .. 0.005 show good results.

## Fine tuning

Once I got the car drive several laps without leaving the road I decided to try Twiddle algorithm presented in the Udacity course. You can find my implementation in the file main.cpp.

It starts with Kp = 0.150, Ki = 0.014, Kp = 2.000 and initial delta chosen to be 1/5 of these values. On every iteration the code ignores first 200 frames to let vehicle stabilize and then tries to find combination of parameters that minimizes the loss function. As loss function I take the sum of cte squares. The algorithm tries to add and then subtract delta to the hyperparameter and chooses the option that improves minimal error. If none of tries could beat the best error the algorithm switches to the next parameter. When all 3 parameters are processed the optimization starts again.
One lap has about 1075 frames, so I take 2350 = 2*1050 + 200 as iteration size.

# Optimization results

After running optimization for a while I got the best results with 

Kp = 0.1625
Ki = 0.0063
Kd = 1.6400


## Improvements

The car drives the map without leaving the road but sometimes it makes very sharp turns. I updated the loss function to also consider the speed of the car in Y direction (delta cte) to penalize intensive steering.          