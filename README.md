![Bilby Stampede](https://cdn.discordapp.com/attachments/794885191898365975/1081016440905277460/bishopLogo_3.png)

## Useful Documentation
[CAN ID Spreadsheet](https://docs.google.com/spreadsheets/d/1NtnqaaMVDYO0TyJ946Wxg0dBtV19xBe5mVzWcAWxIAw/edit#gid=1456793576)

[Motors and Gearing Ratios - Bishop](https://docs.google.com/spreadsheets/d/1mly-FWH9S1RMrAUBcaXnyuavnCqU-cXk0Q0pLDEhZ-Y/edit#gid=1544976692)

[Motors and Gearing Ratios - Windsor](https://docs.google.com/spreadsheets/d/1FxBIIsZFDOvoKsso25b7TmFgGUk4gB1KhH03Lld9y3U/edit#gid=1544976692)

[Driveteam button mapping](https://docs.google.com/document/d/1LmwfAIl3pLnZguX8B4lljc1ZuzqiQKjrft7fehE6e5s/edit)

[Auto routines diagram](./src/main/java/frc/robot/commands/auto/doc/Auto%20Routines%20Diagram.pdf)

## Libraries 
[LimelightHelpers](https://github.com/LimelightVision/limelightlib-wpijava)

[Pheonix](https://store.ctr-electronics.com/software/)

[Rev](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information)

## Button Bindings
<p align="center"><b>Comp Configuration<b><p>

| Binding | Controller | Subsystem/Command | Function | Toggle/Push | 
|:-------:|:----------:|:------------------|:---------|:-----------:|
|RT|1|Drivetrain|Accelerate|Push|
|LT|1|Drivetrain|Reverse|Push|
|LB|1|Drivetrain|Precision mode|Push|
|RB|1|Drivetrain|Precision mode|Push|
|LSB-X|1|Drivetrain|Left-Right|Push|
|LB|2|Limelight|Target node|Push|
|DPAD-UP|2|Arm|Place cube top|Push|
|DPAD-DOWN|2|Arm|Place cube bottom|Push|
|X|1|Arm|Grab loading zone|Push|
|A|1|Arm,Intake|Intake handoff|Push|


<p align="center"><b>Demo Configuration<b><p>

| Binding | Controller | Subsystem/Command | Function |
|:-------:|:----------:|:------------------|:---------|
|Y|TBD|Intake|Pivot up|
|A|TBD|Intake|Pivot down|
|X|TBD|Intake|Roller inwards|
|B|TBD|Intake|Roller outwards|


## CAN ID's
| CAN | CAN Bus | Component | Subsystem(s) |
|:---:|:--------|:----------|:-------------|
|2|rio|PDP|-
|3|rio|PCM|-
|10|rio|Pigeon 2.0|*DR*
|19|rio|CANdle|-
|20|drive|Falcon Left 1|*DR*, *LLR*
|21|drive|Falcon Left 2|*DR*, *LLR*
|22|drive|Falcon Left 3|*DR*, *LLR*
|22|rio|Shoulder spark-Max*|-
|23|drive|Falcon Right 1|*DR*, *LLR*
|24|drive|Falcon Right 2|*DR*, *LLR*
|24|rio|Elevator Spark-Max|*ELA*
|25|drive|Falcon Right 3|*DR*, *LLR*
|29|drive|CANCoder Right|*DR*
|30|drive|CANCoder Left|*DR*
|34|x|Falcon Intake 1|*ITK*
|28|x|Neo Intake 1|*ITK*
|x|x|Neo Intake 2|*ITK*

*CAN ID's are not finalized and are subject to change*

## Subsystems
|Abbrv|Subsystem|Purpose| 
|:---:|:--------|:------|
|DRV|Drivetrain|Driving|
|ITK|Intake|Intake|
|LLA|LimeLight April-Tags|V/ision|
|LLR|LimeLight Retro-Reflective|Vision|
|ELA|ElevatorArm|Telescopic Arm|

<p align="center">(<a href="#readme-top">back to top</a>)</p>
