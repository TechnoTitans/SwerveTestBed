# SwerveTestBed
FRC 1683's code repository for an empty swerve drivetrain.

![CI](https://github.com/TechnoTitans/SwerveTestBed/actions/workflows/CI.yml/badge.svg)

## Structure
```
src - RobotCode
├── main - Functional code, deployed to the robot
│   ├── deploy - Files/resources deployed to the robot (not main RobotCode)
│   ├── java/frc/robot - Primary RobotCode root
└── test/java - Unit/Integration tests
    ├── frc/robot - Test classes
    └── testutils - Test utilities (if present)
```

### Always Keep Cooking...
1. Fix auto!!!
2. Re-fix auto!!
3. Break auto...
4. Fix auto again!
5. Repeat
