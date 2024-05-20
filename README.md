# TOOTHLESS

## Background

FRC 5881 1st annual off season programming project. In this project we're going back in time, to the 2017 [FRC STEAMWORKS](https://www.youtube.com/watch?v=EMiNmJW7enI) competition. The project is to program a simulated FRC Robot. This is an individual project. All you will be given are some simulated motor controllers and mechanisms. You will be responsible for creating the Abstractions and using the correct control loops to make the robot move.

In addition to whatever programming support you need, the mentors will also serve as the "build team". Just like a real FRC season, part of programming team's responsibility is to communicate with the build team to tease out the robot requirements and constraints. Some parts of this project have been intentionally left vague with the expectation that you should ask questions.

Before getting into the details of the robot it's important to watch the game animation <https://www.youtube.com/watch?v=EMiNmJW7enI>.

### Systems

At a high level the robot has the follow mechanisms:

- KOP Style Differential (Tank) Drive
- Ground Intake for FUEL that has dual use as a winch for climbing
- Toggleable Pneumatic ramp/shield for collecting GEARs from the LOADING STATION or FUEL from the HOPPERS
- FUEL Launcher for scoring HIGH in the BOILER
- FUEL Indexer that feeds the FUEL Launcher
- FUEL Agitator that sits under the robot's storage area to encourage indexing

Following the completion of this robot another mechanism will be added.

## Setup

1. Update WPILib to version 2024.3.2, the latest version at the time of writing.

2. Clone this repository.

3. Create a new branch for your individual work. Your branches should be prefixed with your initials. For example, my name is Christopher Mahoney so I would name my main branch `cm/main`. Feel free to create as many branches as you want under your namespace.

4. Read `robotPeriodic` in `Robot.java`, and `Simulation.java` to see how the robot is simulated.

5. Read <https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html> to understand how to run the simulation and setup the GUI.

From here you should start writing Subsystems and Commands. You should also start asking questions about requirements and constraints.

### GUI Example

![GUI Example](image.png)