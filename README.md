# CarND-MPC-Controller
Drive a race car around a track using a MPC controller.  
The program is written in C++.  This Project is from Udacity's Self-Driving Car Engineer Nanodegree Program.

## Basic Set-up
1. Clone this repo.
2. Install the libraries uWebSockets and Ipopt 
..* uWebSockters: Make sure [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) is installed.  Two install scripts are included for MAC and Linux.  For Windows use [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) and following the Linux instructions.
..* Ipopt: Make sure Ipopt [Ipopt](https://projects.coin-or.org/Ipopt) is installed. The install script istall_ipopt.sh is included along with instructions that can be found in install_Ipopt_CppAD.md.
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run the programs: Run `./mpc`. Open [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) and run the corresponding page.  

## The Model
To drive the race car around the track, we need to specify its state variables, its actuators and the motion equations.
#### The State
#### The Actuators
#### The motion equations

#### How we evole the model.

#### Accounting for latency.

#### Adjusting the model.


## The hyperparameters

## Other Important Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
