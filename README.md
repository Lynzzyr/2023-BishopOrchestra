# 2023-Bishop

## Useful Documentation
[CAN ID Spreadsheet](https://docs.google.com/spreadsheets/d/1NtnqaaMVDYO0TyJ946Wxg0dBtV19xBe5mVzWcAWxIAw/edit#gid=1456793576)

[Motors and Gearing Ratios - Bishop](https://docs.google.com/spreadsheets/d/1mly-FWH9S1RMrAUBcaXnyuavnCqU-cXk0Q0pLDEhZ-Y/edit#gid=1544976692)

[Motors and Gearing Ratios - Windsor](https://docs.google.com/spreadsheets/d/1FxBIIsZFDOvoKsso25b7TmFgGUk4gB1KhH03Lld9y3U/edit#gid=1544976692)

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
|LLA|LimeLight April-Tags|Vision|
|LLR|LimeLight Retro-Reflective|Vision|
|ELA|ElevatorArm|Telescopic Arm|
