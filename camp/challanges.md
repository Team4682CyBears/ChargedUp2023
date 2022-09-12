[[_TOC_]]

# FRC Code / Electrical Bootcamp
CyBears coders new to FRC will ideally go through training to learn how FRC electronics and code work together.  The general idea behind a boot camp is to accelerate learning.  To attempt to take advantage of a bit of rapid learning, CyBears will therefore attempt to have each coding team member work through a key 'example project'.

The idea behind an example project will have each individual project interact with some form of input and have that signal control a form of robot output.  Successful completion of the project includes both checking in of the full project into the ChargedUp2023\camp\{team member} repo directory as well as demonstrating the functionality checked in works on the project test bed.

# Input and Output Pairings / Assignments

The table below highlights various challanges that coder members will be assigned as their bootcamp example project.

|Input Hardware | Output Hardware | Expected Outcome | Owner |
|--|--|--|--|
| XBox Controller Left Stick Y direction | Talon motor  | Use of the input of the Left stick y double/floating point value to control the speed of the motor | ?? |
| Multi-function joystick twist | Neo Motor | Use of the input of the joystick twist axist double/floating point value to control the speed of the motor | ?? |
| Button board two buttons | bag motor on/off in forward and reverse directions | Use one button to drive the motor in forward direction and the other button press to run the motor in the other direction | ?? |
| XBox Controller D-Pad up/down | Pneumatic double soleniod enable/disable | The intent here is to trigger a pneumatic solenoid using the xbox controller D-pad input as boolean values. | ?? |
| Beam break sensor | bag motor on/off in a single direction | Every time a beam break sensor is broken turn motor on, otherwise turn off motor | ?? |
| Distance sensor | Small neo motor? | As something comes into view of the sensor closer and closer motor speed increases.  When nothing is in view the motor is off | ?? |
| NavX various movements | Musical notes functionality on a Talon motor | Every time an axis movement on the navx occurs one of 6 notes is played on the Talon. | ?? |


# Expected Activity Breakdown

The sections below generally describe the steps that should be followed during bootcamp.

## Prerequisites

All team members should follow the getting starting guide.  See: [2023 Getting Started Guide](https://github.com/Team4682CyBears/ChargedUp2023/blob/main/docs/ChargedUp2023_Code_GettingStarted.docx)

##  Sign Up As the Owner for an Example Project

1. Create a feature branch from the [ChargedUp2023 Repo](https://github.com/Team4682CyBears/ChargedUp2023)
2. Make an edit to this file [challanges.md](https://github.com/Team4682CyBears/ChargedUp2023/blob/main/camp/challanges.md) and add your name as the 'owner' in one of the unassigned '??' above 
3. Create a pull request with the change
4. Have a mentor approve your pull request
5. Merge the PR

## Build an Example Project Capable of Meeting the Expected Outcome

1. Create a New WPILib Project
    a. by using one of the WPILib 'command' based examples 
       i. Select Command Example 
       ii. Motor Control with Encoder
    b. select project directory - e.g., %root%/camp/username
    c. Name your project with something that reflects the expected outcome
    d. Team Number - 4682
    e. Click Generate Project
2. Import External Packages
3. Research expected APIs that will be needed
4. Add appropriate command classes
5. Add appropriate subsystem classes
6. Complie
7. Deploy
8. Test