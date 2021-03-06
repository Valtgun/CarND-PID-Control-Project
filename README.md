# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection
### Effect of each of the P, I, D components
P - proportional term was the main parameter that contributes to the control. It has direct response to the current error value.
I - integral term is accumulating error over time, it is used when P term is not correcting sufficiently to reduce the error. In case of understeering I part becomes more significant and corrects the steering control.
D - derivative term is used to reduce overshooting and therefore makes smoother behavior. If the value is too large then car will behave too smoothly also in the corners and will not be able to steer successfully in the corners.

PID controller once parameters were tuned performed very well. It can even compare with the ConvNet approach in terms of driving behavior. However, PID controller requires ground truth, which is not easily available in realistic situations, which make it worse in terms of real world applications.

### How the final hyperparameters were chosen
Final hyperparameters for submission was tuned manually. However, during the process also there were experiments with Twiddle approach which is described in next chapter.

For the manual tuning, it was done in multiple step process (as described in https://en.wikipedia.org/wiki/PID_controller#Manual_tuning):
As the proportional term is most important in the PID controller then initially only P parameter was changed (with I and D set to 0 coefficient).
Once P controller was starting to oscillate it the next I parameter was tuned.
During this process, if P parameter was set too low, then car was not able to steer back to the centerline and if it was too large value then oscillations increased and it run out of track at some point.

As the next, the Integral term was adjusted. It was used to get the result when car remains on the track, with the oscillations present. Once car was able to remain on track the D term was adjusted.

Derivative term was adjusted as last and is necessary to reduce oscillations and make steering controls smoother.

Final parameters were:
P = 0.075
I = 0.005
D = 0.075

### Experiment with hyperparameter tuning using twiddle
During the process Twiddle was used (it is located in test-twiddle branch https://github.com/Valtgun/CarND-PID-Control-Project/tree/test-twiddle).
There were multiple complexities when tuning using twiddle.
Main issue was that with existing simulator there were no solution to Reset car back to initial state. And if the car was not reset then it was difficult to compare results over different runs as the track contained varying number of corners and straight runs.
It was interesting to test and see the challenges, but it did not allow better tuning of parameters.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/aca605f8-8219-465d-9c5d-ca72c699561d/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/12dd29d8-2755-4b1b-8e03-e8f16796bea8)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
