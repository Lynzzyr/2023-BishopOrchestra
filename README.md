![Bishop](./doc/bishopLogo.png)

## Useful Documentation
[CAN ID Spreadsheet](https://docs.google.com/spreadsheets/d/1NtnqaaMVDYO0TyJ946Wxg0dBtV19xBe5mVzWcAWxIAw/edit?usp=sharing)

[Motors and Gearing Ratios - Bishop](https://docs.google.com/spreadsheets/d/1mly-FWH9S1RMrAUBcaXnyuavnCqU-cXk0Q0pLDEhZ-Y/edit?usp=sharing)

[Motors and Gearing Ratios - Windsor](https://docs.google.com/spreadsheets/d/1FxBIIsZFDOvoKsso25b7TmFgGUk4gB1KhH03Lld9y3U/edit?usp=sharing)

[Driveteam button mapping](https://docs.google.com/spreadsheets/d/1Z_SK2qxh_o4-e56WKn4cpELmqUnG5LR1-ANQ0Ndb_h8/edit?usp=sharing)

[Auto routines diagram](./doc/Auto%20Routines%20Diagram.pdf)

## Libraries 
[LimelightHelpers](https://github.com/LimelightVision/limelightlib-wpijava)

[Pheonix](https://store.ctr-electronics.com/software/)

[Rev](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information)

## Button Bindings

| Button      | Controller | Function                                           | Subsystem(s)          | Action |
| :---------: | :--------: | :------------------------------------------------- | :-------------------- | :----: |
| RT          | Primary    | Accelerate                                         | Drivetrain            | Hold   |
| LT          | Primary    | Decelerate/reverse                                 | Drivetrain            | Hold   |
| LB          | Primary    | Cone node aim                                      | Limelight, Drivetrain | Hold   |
| RB          | Primary    | Gear shifting (mid-high)                           | Drivetrain            | Hold   |
| LS-X        | Primary    | Rotation                                           | Drivetrain            | Hold   |
| LS-Y        | Primary    |                                                    |                       |        |
| LS-B        | Primary    |                                                    |                       |        |
| RS-X        | Primary    |                                                    |                       |        |
| RS-Y        | Primary    |                                                    |                       |        |
| RS-B        | Primary    |                                                    |                       |        |
| X           | Primary    | Auto-close claw for cone                           | Claw, Arm, Telescope  | Hold   |
| Y           | Primary    | Auto-close claw for cube                           | Claw, Arm, Telescope  | Hold   |
| A           | Primary    | Open claw                                          | Claw                  | Press  |
| B           | Primary    | Open-claw to armed position                        | Claw                  | Hold   |
| D-PAD UP    | Primary    | Manual claw open                                   | Claw                  | Hold   |
| D-PAD DOWN  | Primary    | Manual claw close                                  | Claw                  | Hold   |
| D-PAD LEFT  | Primary    |                                                    |                       |        |
| D-PAD RIGHT | Primary    |                                                    |                       |        |
| START       | Primary    | Stall on charge station                            | Drivetrain            | Hold   |
| BACK        | Primary    |                                                    |                       |        |
| RT          | Secondary  | Manual arm up                                      | Arm, CANdle           | Hold   |
| LT          | Secondary  | Manual arm down                                    | Arm, CANdle           | Hold   |
| LB          | Secondary  | Move arm and retract to idling position            | Arm, Telescope        | Press  |
| RB          | Secondary  | Move arm and retract to double substation          | Arm, Telescope        | Press  |
| LS-X        | Secondary  |                                                    |                       |        |
| LS-Y        | Secondary  |                                                    |                       |        |
| LS-B        | Secondary  | Set LED to cone (yellow)                           | CANdle                | Press  |
| RS-X        | Secondary  |                                                    |                       |        |
| RS-Y        | Secondary  |                                                    |                       |        |
| RS-B        | Secondary  | Set LED to cube (purple)                           | CANdle                | Press  |
| X           | Secondary  | Move arm and retract to cone low position          | Arm, Telescope        | Press  |
| Y           | Secondary  | Move arm and extend to top cube position           | Arm, Telescope        | Press  |
| A           | Secondary  | Move arm and retract to mid cube position          | Arm, Telescope        | Press  |
| B           | Secondary  | Move arm and retract ABOVE mid cone node position  | Arm, Telescope        | Press  |
| D-PAD UP    | Secondary  | Manual telescope extend                            | Drivetrain            | Press  |
| D-PAD DOWN  | Secondary  | Manual telescope retract                           | Drivetrain            | Press  |
| D-PAD LEFT  | Secondary  |                                                    |                       |        |
| D-PAD RIGHT | Secondary  |                                                    |                       |        |
| START       | Secondary  | Set LED to red                                     | CANdle                | Press  |
| BACK        | Secondary  | Move arm and retract to resting on intake position | Arm, Telescope        | Press  |

### Button Bindings Legend

| Key         | Value              |
| :---------: | :----------------: |
| RT          | Right trigger      |
| LT          | Left trigger       |
| LB          | Left bumper        |
| RB          | Right bumper       |
| LS-X        | Left stick x-axis  |
| LS-Y        | Left stick y-axis  |
| LS-B        | Left stick button  |
| RS-X        | Right stick x-axis |
| RS-Y        | Right stick y-axis |
| RS-B        | Right stick button |
| X           | X                  |
| Y           | Y                  |
| A           | A                  |
| B           | B                  |
| D-PAD UP    | D-Pad up           |
| D-PAD DOWN  | D-Pad down         |
| D-PAD LEFT  | D-Pad left         |
| D-PAD RIGHT | D-Pad right        |
| START       | Start              |
| BACK        | Back               |
| *empty*     | Unbound button     |


## CAN IDs

| Part                      | ID  | CAN Bus         |
| :------------------------ | :-: | :-------------: |
| PDH                       | 1   | rio             |
| Pigeon 2.0                | 10  | rio             |
| CANdle                    | 19  | rio             |
| Falcon Left 1             | 20  | rio             |
| Falcon Left 2             | 21  | rio             |
| Falcon Left 3             | 22  | rio             |
| Falcon Right 1            | 23  | rio             |
| Falcon Right 2            | 24  | rio             |
| Falcon Right 3            | 25  | rio             |
| CANCoder (Left)           | 29  | rio             |
| CANCoder (Right)          | 30  | rio             |
| Elevator SparkMax         | 38  | rio             |
| Falcon Claw               | 39  | rio             |
| Intake Roller Falcon      | 28  | rio             |
| Left ToF Claw             | 36  | rio             |
| Right ToF Claw            | 37  | rio             |
| Intake Wrist SparkMax     | 34  | rio             |
| Intake Pivot SparkMax     | 35  | rio             |
| Shoulder spark max        | 33  | rio             |
| Shoulder spark max        | 32  | rio             |
