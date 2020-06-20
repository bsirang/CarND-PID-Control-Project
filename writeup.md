# PID Control

## Goal
The goal of this project was to implement a PID controller to use for cross track error steering correction.

## About PID

### P
The P component represents the control response that's proportional to the error signal. The larger the error, the stronger the control response. This can be thought of like a spring, which has a force pushing back that is proportional to linear displacement.

### I
The I component represents the integral of the error over time. It is a low-frequency response meant to compensate for systematic bias, such as a car that is misaligned and doesn't drive perfectly straight even if the wheel is straight.

### D
The D component represents a dampening of the control response. It will temper the control response if our error signal is fast approaching zero. In other words it is proportional to the rate of change of the error. This helps prevent overshoot and provides an additional response if we're falling away from our target.

### Hyperparameters

Here's the list of parameters:

* Kp - Proportional gain
* Ki - Integral gain
* Kd - Derivative gain
* KiLim - Integral wind-up limit
* KLpfAlpha - Derivative low pass filter strength
* KErrorBand - Band in which to consider the error zero

Aside from the standard PID gains, there were a couple other hyperparameters. The `KiLim` parameter is meant to limit the amount that the integral term can wind up. This is to prevent the integral term from growing very large. This may happen, for example, if the vehicle is stopped, thus can't reduce the cross track error no matter what the steering angle is. The `KLpfAlpha` parameter is used to filter the derivative term. If the error signal is noisy, then the noise in the derivative term would become magnified. A low pass filter can help smooth the signal so that it doesn't generate an erratic control response. The `KErrorBand` parameter is used to prevent excessive small corrections when the cross track error is just about zero. If the magnitude of the error is less than this value, it will simply be treated as zero.


## Twiddle
The twiddle algorithm, as described in the course, was implemented and used to find optimal `Kp` and `Kd` gains. I left the `Ki` parameter out of twiddle because I knew intuitively that the P and D terms were most important and I wanted to save time.

I used twiddle starting with initial gains that I knew would at least keep the car on the track, and then I chose a number of samples to use for averaging the error. I chose to use a number of samples large enough to cover a complete lap around the track.

I didn't leave an exit condition for the twiddle algorithm since it was running in real-time. I simply left it running for a long period of time, and then analyzed the results and used them to set new gains.

## Reflections

After spending a fair bit of time trying to identify optimal gains, I found some gains that work well enough to keep the vehicle on the track. However, no matter what gains I tried, the vehicle would always jerk the wheel at least a little bit, especially around turns.

After thinking about the nature of the controller and the error signal being cross track error, I convinced myself that this is a fundamental limitation of using a PID controller in this way, given that the PID controller is purely reactive and not future looking.

Imagine the vehicle is turning around a curve, when the vehicle corrects the steering and reduces the cross track error to 0, the vehicle will straighten the wheel completely. This will inevitably cause the error to increase and another correction to occur. This was mitigated slightly by using the integral term to help keep some steering response.

I believe the key to solving that problem is the incorporate a feed-forward term, which is a function of the curvature of the road. In other words, if we understood the curvature of the road, our steer angle can be set to the expected angle based on the curvature of the road, and the PID controller would simply add a control response on top of that feed-forward term.

Without this feedfoward term, the best I could do was to use the `I` term to compensate for road curvature. But that meant I had to make sure the `I` term could wind up and down quickly enough to still be respond to S curves.
