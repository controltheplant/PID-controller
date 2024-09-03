# PID-controller repository
------
## Table of contents

1. [Preface](https://github.com/controltheplant/PID-controller/blob/main/readme.md#1--preface)\
    1.1. [About this repository](https://github.com/controltheplant/PID-controller/blob/main/readme.md#11--about-this-repository)\
    1.2. [Target audience](https://github.com/controltheplant/PID-controller/blob/main/readme.md#12--target-audience)\
    1.3. [Structure of repository](https://github.com/controltheplant/PID-controller/blob/main/readme.md#13--structure-of-repository)
2. [PID_Controller (FB)](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2--pid_controller-fb)\
    2.1. [Short description](https://github.com/controltheplant/PID-controller/blob/main/readme.md#21--short-description)\
    2.2. [Block interface](https://github.com/controltheplant/PID-controller/blob/main/readme.md#22--block-interface)\
    2.3. [Functional description](https://github.com/controltheplant/PID-controller/blob/main/readme.md#23--functional-description)\
          2.3.1. [PID Algorithm](https://github.com/controltheplant/PID-controller/blob/main/readme.md#231-pid-algorithm)\
          2.3.2. [Functional diagram](https://github.com/controltheplant/PID-controller/blob/main/readme.md#232-functional-diagram)\
          2.3.3. [List of functions](https://github.com/controltheplant/PID-controller/blob/main/readme.md#233-list-of-functions)\
          2.3.4. [Scaling](https://github.com/controltheplant/PID-controller/blob/main/readme.md#234-scaling)\
          2.3.5. [Modes](https://github.com/controltheplant/PID-controller/blob/main/readme.md#235-modes)\
          2.3.6. [Bumpless transfer](https://github.com/controltheplant/PID-controller/blob/main/readme.md#236-bumpless-transfer)\
          2.3.7. [Filters](https://github.com/controltheplant/PID-controller/blob/main/readme.md#237-filters)\
          2.3.8. [Rate limit](https://github.com/controltheplant/PID-controller/blob/main/readme.md#238-rate-limit)\
          2.3.9. [Inversion of control direction](https://github.com/controltheplant/PID-controller/blob/main/readme.md#239-inversion-of-control-direction)\
          2.3.10. [Deadband](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2310-deadband)\
          2.3.11. [Offset](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2311-offset)\
          2.3.12. [Anti-windup](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2312-anti-windup)\
          2.3.13. [Limit](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2313-limit)\
	2.4. [Referenced objects](https://github.com/controltheplant/PID-controller/blob/main/readme.md#24-referenced-objects)\
	2.5. [Change Log](https://github.com/controltheplant/PID-controller/blob/main/readme.md#25-change-log) 
------
## 1  Preface

### 1.1  About this repository

*PID-controller* is a repository created to show the functionality of the PID controller, which is widely used in industrial automation. In the future, other software blocks used in the automation of production processes may be added to this repository. Project [controltheplant](https://github.com/controltheplant) is maintaining this repository. The software blocks hosted in this repository are offered free of charge. The software blocks are provided "as is" without any warranties. For more details, see the License conditions.

### 1.2  Target audience

Software blocks hosted in this repository are intended for qualified automation professionals. Usually, a degree in such areas as industrial automation, control systems, instrumentation is required. Knowledge of the architecture of industrial control systems, PLCs, control theory is required. The source code is written in Codesys V3.5 SP20 IDE.

### 1.3  Structure of repository

This repository includes all the subblocks required for the `PID_Controller` function block. Those blocks are located in the *'/sub blocks'* directory. These blocks are:
- `DeadBand` function implements a dead zone nonlinear element;
- `RateLimiter` function block limits the rate of change of the input signal according to a linear ramp function;
- `SCALE_R` function scales the input value from the input range to the output range;
- `T_PLC_US` function returns the current PLC system time in microseconds;
- `MovingAverage` function block calculates the average value of the specified number of previous historical values of input value;
- `FindSteadyState` function block calculates whether the input value is stable, meaning the variation of the input value does not exceed a given value.
- `LSim_PT1` function block. Same as `LSim_PT1` block in *'/sim'* directory.

User data types (UDTs in Siemens terminology) or Data Unit Types (DUTs in CoDeSys terminology) used in software blocks are located in *'/types'* directory along with enumerations. They are:
- `enumAntiWindupMethods` is an enumeration of anti-windup methods provided in `PID-controller` function block;
- `enumBumplessTransferMethods` is an enumeration of bumpless transfer methods from auto to manual modes and vice versa, provided in `PID-controller` function block;
- `typePIDSettings` is a structure containing specific settings of the PID controller.

To test the `PID-controller` function block, a mathematical model of controllable object is required. Blocks used to build the model of the controllable object are located in *'/sim'* directory, including:
- `LSim_Lagging` function block. It implements the time delay of the input signal;
- `LSim_PT1` function block. It implements the first-order low-pass filter, or aperiodic element, or PT1 element;
- `LSim_IT1` function block. It implements the real integrating element, which is an integrator with first-order delay, or IT1 element;
- `NoiseGenerator` function block. It is used to simulate the white noise that affects an analog signal. White noise is simulated as a sequence of random numbers within a specified range.

## 2  PID_Controller (FB)
<sup>Engineering environment: Codesys V3.5 SP20</sup>\
<sup>Author: controltheplant project</sup>

### 2.1  Short description

This block implements a proportional-integral-derivative controller for industrial control systems.

> [!IMPORTANT]
> Function block must be called in a cyclic interrupt

### 2.2  Block interface

#### Input parameter
| Identifier | Data type | Default value | Description |
|-----        |:----:|:-----:|-----|
|setpoint     | REAL |       | Setpoint in auto mode, [eng. units]|
|actualValue  | REAL |       | controllable (process) value used as feedback, [eng. units]|
|actuatorPos  | REAL | 0.0   | actual measured position of the actuator, [0..100%]. Set 0 if not used|
|offset       | REAL | 0.0   | disturbance compensation or precontrol value, [0..100%] |
|trackingValue| REAL |       | setpoint applied in tracking mode, [0..100%] |
|track        | BOOL | FALSE | enable tracking mode|
|manual       | BOOL | TRUE  | enable manual mode |
|hold         | BOOL | FALSE | "freeze" controller's output|
|reset        | BOOL | FALSE | reset output to predefined value and reset errors|
|rangeHi      | REAL | 10.0  | actual value upper measurement limit, [eng. units]|
|rangeLo      | REAL |  0.0  | actual value lower measurement limit, [eng. units]

#### Output parameter

| Identifier | Data type | Default value | Description |
|-----|:-----:|:-----:|-----|
|out| REAL | |controller's output (CO), [0..100%]|
|limitsActive | REAL | | 1=upper or lower limit of output reached or rate of change of controller's output exceeded (if specified) |
|error| BOOL | FALSE | block issued an error |
|errorID| DWORD | 0 | error code |

#### In/Out parameter

| Identifier | Data type | Description |
|-----|:-----:|-----|
|manualValue| REAL | setpoint for manual mode; in auto mode it follows the controller's output |

#### Static parameter
| Identifier | Data type | Description |
|-----|:-----:|-----|
|settings| typePIDSettings | structure containing advanced PID settings |

#### User-defined datatypes
#### typePIDSettings

| Identifier | Data type | Default value | Description |
|-----|:-----:|:-----:|-----|
|iwAntiWindupMethod      | enumAntiWindupMethods       | | method selected to prevent integrator saturation|
|iwBumplessTransferMethod| enumBumplessTransferMethods | | method selected for transfer from Manual to Auto|
|irKP                 | REAL | 1.0   | proportional term. Value must be positive |
|irTI                 | REAL | 0.0   | integral term. Negative value is forbidden|
|irTD                 | REAL | 0.0   | derivative term. Negative value is forbidden|
|irTdF                | REAL | 0.5   | Filter time for derivative component |
|ixInverted           | BOOL | FALSE | inverted controller action |
|irDeadband           | REAL | 0.0   | deadband for error signal, [%] |
|itSetpointFilterTime | REAL | 0.0   | time const for setpoint filter, [sec]|
|irSetpointRampRate   | REAL | 0.0   | rate of change of setpoint, [1/sec]|
|itActValueFilterTime | REAL | 0.0   | time const for actual value filter, [sec]|
|irOutRampRate        | REAL | 0.0   | rate of change of controller output, [1/sec]|
|irDeviationTolerance | REAL | 1.0   | control deviation at which the transition process is considered complete, [%]|
|irStabilizationTime  | REAL | 2.0   | time after which the process is considered stable, if it is inside the dead zone, [sec]|
|irOutLimitL          | REAL | 0.0   | lower limit of controller's output, [%]|
|irOutLimitH          | REAL | 100.0 | higher limit of controller's output, [%]|
|irBackCalcFactor     | REAL | 1.0   | time const for back-calculation anti-windup, [sec] |
|irResetValue         | REAL | 0     | value applied when reset is active |

#### enumAntiWindupMethods

| Identifier | Default value| Description |
|-------|:------------:|------------|
|CLAMPING        |  X  | integration stops when saturation is reached |
|BACK_CALC_MODEL |     | backward calculation, ramp function used as actuator model |
|BACK_CALC_REAL  |     | backward calculation, measured actuator position used |

#### enumBumplessTransferMethods

| Identifier | Value| Description |
|-------|:------------:|------------|
|TRACK_SETPOINT    | 0 | SP tracks the PV. After switching to auto SP returns to the previous value using rate limiter. Setpoint rate limiter must be non-zero! |
|SUPPRESS_P_ACTION | 1 | P-action of the controller is suppressed until the process stabilizes. Only Integral action. Recommended if you don't use a setpoint rate limiter |

#### Constants
|Identifier & value| Description|
|-----|-----|
| **MICROSECONDS_IN_SEC** <br /> 1.0E-6 | coefficient to convert microseconds to seconds |
| **EPS**                 <br /> 1.0E-44| smallest real value                            |

#### Status & Error codes

|Code / Value | Identifier / Description |
|-----|-----|
|  0     | STATUS_NO_ERRORS <br/> no errors                                                      |
|16#8201 | ERR_BAD_LIMITS <br/> bad low or/and high limits                                       |
|16#8202 | ERR_CANT_TRACK_SETP <br/> 'TRACK_SETPOINT' method selected, but setpoint ramp is zero |
|16#8203 | ERR_BAD_RATE <br/> rate of change is negative                                         |
|16#8001 | ERR_NO_CYCLE <br/> PLC cycle time is negative or zero                                 |
|16#8205 | ERR_BACK_CALC_T <br/> bad back calculation factor                                     |
|16#8207 | ERR_BAD_ANTIWINDUP <br/> error in anti-windup settings                                |
|16#8209 | ERR_BAD_STAB_TIME <br/> stabilization time should be positive                         |
|16#820A | ERR_BAD_DEV_TOL <br/> deviation tolerance must be in the range from 0 to 50%          |
|16#820B | ERR_BAD_PID_COEFFS <br/> wrong PID parameters                                         |

------
### 2.3  Functional description

#### 2.3.1 PID Algorithm

The function block is a PID controller with continuous output signal (manipulated variable). Its purpose is to activate a final controlling element with continuous action input. PID controller continuously acquires the measured process value within a control loop and compares it with the required setpoint. Based on the resulting control deviation, the function block `PID_Controller` calculates an output value by which the process value is adapted to the setpoint as quickly and stable as possible. The output value for the PID controller consists of three actions:
- **Proportional** action\
The proportional action of the output value increases in proportion to the control deviation;
- **Integral** action\
The integral action of the output value increases until the control deviation has been balanced. The trapezoid method is used to calculate the integral part;
- **Derivative** action\
The derivative action increases with the rate of change of control deviation. The process value is corrected to the setpoint as quickly as possible. The derivative action will be reduced again if the rate of change of control deviation drops. The derivative part is implemented as a DT1 element.

The `PIC_Controller` block implements **velocity-based** algorithm,  also known as **incremental** algorithm. **Ideal** (**ISA Standard**) form of PID algorithm has been chosen. This gives the following advantages:
- compatibility with legacy systems;
- compliance with ISA Standard form;
- ease of implementation of such functions as bumpless transfer, anti-windup protection, rate limit;
- ease of upgrading to a 3-step PID controller to drive integral-type actuators, like motor-driven valves.

The internal structure of the PID Controller is given in the form of a block diagram of discrete z-transfer functions, which is shown in Figure 1:

|![figure1](https://github.com/user-attachments/assets/73fbbf0f-c6ae-4083-bba0-54fce8ff29fb "Block diagram of PID Controller")|
|:-----:|
|Figure 1 – Block diagram of PID Controller|

the figure indicates:\
\- *z* – operator of z-Transform;\
\- *T* – cycle time of `PID_Controller` block or sample period;\
\- *K<sub>P</sub>* – proportional gain;\
\- *T<sub>I</sub>* – integration time;\
\- *T<sub>D</sub>* – derivative time;\
\- *T<sub>f</sub>* – filter time constant for derivative part.

The block diagram shown in Figure 1 is simplified. Some blocks like dead zone, anti-windup, filter are not shown here for simplicity.

> [!TIP]
> Always use filtration for the derivative part because differentiation greatly amplifies the signal noise. Set the `settings.irTdF` parameter for this.

#### 2.3.2 Functional diagram

Functional diagram of `PID_Controller` block is shown on Figure2:

| ![PID Functional scheme](https://github.com/user-attachments/assets/01ff8e79-5f16-4953-a689-a21012cad1f5) |
|:------:|
|Figure 2 - Functional diagram of PID controller|

#### 2.3.3 List of functions

The `PID_Controller` function block has the following functions:
- **Scaling** of input and output values;
- Auto, manual, track, hold and reset **modes**;
- **Bumpless transfer** between Auto mode and other modes and vice versa. Two different methods are available to select from;
- **Filtration** of setpoint and actual values;
- **Rate limit** for setpoint and output signal;
- **Inversion** of control direction;
- **Deadband** for control deviation;
- **Offset** input for feedforward control and disturbance rejection;
- **Anti-windup** function with three different methods available to select from.
- **Limit** the range of the output signal.

#### 2.3.4 Scaling

Setpoint, actual value, and controller output must be scaled to the same range. This is necessary to avoid the influence of the change in the process value measurement range on the control loop gain. The setpoint, actual value, and controller output are always scaled to an internal range from 0 to 100%. The range of setpoint and actual value is always the same. That range should be specified in the `rangeLo` and `rangeHi` [inputs](https://github.com/controltheplant/PID-controller/blob/main/readme.md#input-parameter) of the block. Values of `rangeLo` and `rangeHi` parameters have to be specified in engineering units (bar, m<sup>3</sup>/s, etc.).

#### 2.3.5 Modes

The `PIC_Controller` function block can work in the following modes:
- auto;
- manual;
- tracking;
- hold;
- reset.

##### Auto mode

In auto mode, the PID controller calculates the controller's output based on setpoint and actual value with the law described in [PID Algorithm](https://github.com/controltheplant/PID-controller/blob/main/readme.md#231-pid-algorithm) section. Set the `manual` input of the block to a `FALSE` value to activate Auto mode.

##### Manual mode

In Manual mode operator can set the controller's output directly. The controller's output is determined by the [`manualValue`](https://github.com/controltheplant/PID-controller/blob/main/readme.md#inout-parameter) parameter of the function block. The setpoint and actual value don't affect the controller's output in this mode. Bumpless transfer between Auto and Manual modes is ensured. When in Auto mode, the manual setpoint follows the controller's output. This is why the `manualValue` parameter is placed in the 'In/Out' section of the block interface. [**Bumpless transfer**](https://github.com/controltheplant/PID-controller/blob/main/readme.md#236-bumpless-transfer) from Manual to Auto mode is described in the corresponding section. Set the `manual` input of the block to a `TRUE` value to activate Manual mode.

##### Tracking mode

Tracking mode is similar to Manual mode with the only difference being that the setpoint for Tracking mode is set separately by external software, not by the operator. In this mode `trackingValue` is transferred to the controller's output. The bumpless transfer is only provided when switching from Tracking to Auto mode. Set the `track` input of the block to a `TRUE` value to activate Tracking mode. The setpoint for Tracking mode is specified via the `trackingValue` input of the block. 

##### Hold mode

Hold mode fixes the current value of the controller's output, which remains constant regardless of any setpoints and actual value. Hold mode is active when the `hold` input of the block is set. This mode can be used, for example, in cascade control to prevent integrator windup. When switching from Hold mode to Auto mode, bumpless transfer is ensured.

##### Reset

Reset mode is used to reset the errors that occurred in the `PID_Controller` block and reset the accumulated integrator sum to a predefined value, which is specified via `settings.irResetValue`. When the `reset` input is activated, the `error` and `errorID` outputs are cleared and the controller's output is set to a `settings.irResetValue` immediately.

> [!WARNING]
> Bumpless transfer doesn't work in Reset mode. A jump in the controller's output value is inevitable when activating the Reset mode.

#### 2.3.6 Bumpless transfer

Bumpless transfer means that when switching between modes, there is no jump in the controller's output value. Bumpless transfer from Auto to Manual mode is achieved by making the `manualValue` follow the controller's output value. Transferring from Auto mode to other modes (tracking, hold, reset) is not bumpless, i.e. it has to be done using external software.

There are two methods of bumpless transfer from Manual mode to Auto, that could be selected by the user. A particular method has to be specified in [`settings`](https://github.com/controltheplant/PID-controller/blob/main/readme.md#typepidsettings) structure of the block's interface, in the field `iwBumplessTransferMethod`. Please refer to the [`enumBumplessTransferMethods`](https://github.com/controltheplant/PID-controller/blob/main/readme.md#enumbumplesstransfermethods) data type.

##### Setpoint tracking

The first method is tracking the setpoint. Its point is that in manual mode the setpoint follows the actual value, so the control deviation is zero at the moment when the block switches to Auto mode. However, the setpoint is transmitted to the block through the input section of the block interface and cannot be changed. That contradiction is resolved by making the setpoint return to the original value at the block input using a rate limiter. Make the following assignment to select this method:
```
settings.iwBumplessTransferMethod := enumBumplessTransferMethods.TRACK_SETPOINT;
```
> [!CAUTION]
> Always use the rate limit function for the setpoint when the `TRACK_SETPOINT` method is selected for bumpless transfer. Failure to do so will result in a transient process after switching to Auto mode and block error. Set the `settings.irSetpointRampRate` field to a non-zero positive value.

##### Suppress the P-action

The second method is suppressing the proportional part of the PID controller. The jump when switching to Auto occurs due to the proportional part of the PID controller. The idea of this method is to suppress the proportional part after switching to Auto mode and work with the integrating part only until the control deviation becomes close to zero and the P-action can be reactivated. Make the following assignment to select this method:
```
settings.iwBumplessTransferMethod := enumBumplessTransferMethods.SUPPRESS_P_ACTION;
```
> [!NOTE]
> This method can only be used for self-regulating controllable objects, like PT1, PT2 systems. It works badly for integrating objects and statically unstable objects that require P-action to stabilize. Use this method with caution for non-self-regulating objects, simulation is recommended. This method also increases the time of the transient process after switching to Auto mode.

> [!IMPORTANT]
> Parameter `settings.irDeviationTolerance` must be specified for this method. This parameter determines the moment when the P-action will be reactivated. Set it to a value close to the desired control accuracy, usually a few percent.

#### 2.3.7 Filters

Filters are available for both setpoint and actual value channels. The filters can be used for suppression of the signal noise. To activate the setpoint filter, set the `settings.itSetpointFilterTime` parameter to a value in seconds. To activate the actual value filter, set the `settings.itActValueFilterTime` parameter. To disable filters set their filter time parameters to zero. These filters are implemented as first-order delay elements or PT1 elements. Keep in mind that filtering in the feedback channel will lead to the reduction of phase stability margin, decreased control dynamics, and potentially incorrect information about the actual value signal.

#### 2.3.8 Rate limit

The rate limit function limits the rate of change of the input value. It results in smoothing of input value jumps and a decrease in the rate of change of the signal. Rate limiters are available in the setpoint channel and controller output. Set the `settings.irSetpointRampRate` parameter to a non-zero value to activate the setpoint rate limiter and the `settings.irOutRampRate` parameter to activate the rate limiter for the controller output. The rate limiter in the controller output channel is active in all controller modes.

> [!TIP]
> You can use a rate limiter for controller output to prevent the damage of controllable object due to a high rate of change of the control signal, for example, to prevent the water hammer effect. When the maximum rate of change of controller output is reached, block output `limitsActive` is set. The rate limiter is a nonlinear element and can lead to instability of the control loop and failure of the actual value to reach the setpoint. I recommend adjusting the PID tuning softer when block output `limitsActive` is blinking or constantly on.

#### 2.3.9 Inversion of control direction

For some processes related to the decrease of material balance or energy, a negative control gain is necessary. Examples of such processes may include cooling, level control with a control element on a drain line, vacuum control. Such a process can be identified as follows: when the controller's output value increases, the process variable decreases. However, the negative control gain is forbidden for this block. Use the `settings.ixInvert` bit in this case. Set this bit to `TRUE` to invert the control direction.

#### 2.3.10 Deadband

Even when the control deviation is close to zero and the actual value reached the setpoint, the noise of the measured actual value signal can lead to small changes in the manipulated variable (controller output), which leads to small movements of the actuator. Quantization noise, integrating behavior of controllable object may also lead to oscillations of the controller output signal. The deadband function can be used to avoid unwanted small movements of the actuator in a steady state of manipulated variable, and thus reduce the energy consumption and wear of the actuator. The transfer function of the Deadband element is shown in the Figure 3:

<p align="center">
    <img src="https://github.com/user-attachments/assets/eedcce23-a2bc-4406-988d-3bb8895fe630"  alt="Deadband transfer function"><br>
    Figure 3 - Transfer function of Deadband element
</p>

Deadband width is specified via the `settings.irDeadband` parameter. When this parameter is set to zero, the deadband function is inactive. If a negative value is set in this parameter, the Deadband function will work with the absolute value.

> [!NOTE]
> Deadband is a non-linear element that significantly affects control quality. It can reduce control accuracy or even make the actual value impossible to reach the setpoint.

There are several disadvantages when the Deadband function is active:
- actual value doesn't reach the setpoint itself, but only the edge of the deadzone, since the control deviation becomes zero at the edge of deadzone;
- actual value may settle into steady states that significantly differ from the setpoint for extended periods of time. If that steady states occurs at the edge of the dead zone, even the smallest deviations will trigger the control intervention, i.e. wear of actuator and energy consumption;

To overcome these drawbacks, adaptive activation and deactivation of the dead zone is provided. Deadzone is activated when the absolute value of control deviation becomes less than the `settings.irDeadband` value for the time specified in the `settings.irStabilizationTime` parameter. In addition to that, the controller output is set to a moving average of its previous values. This helps to settle actual value close to the middle of the dead zone, and not at the edge of it. Deadband is temporarily deactivated when a large control deviation occurs (i.e. dead zone is exited) and reactivated again when the controller returns the actual value to the proximity of the setpoint. The transition process is thus freed from the negative influence of the dead zone.

> [!TIP]
> Set the `settings.irDeadband` parameter to a value of 2-3 times of standard deviation of controller output signal oscillations. Tune the `settings.irStabilizationTime` parameter according to the dynamics of your system. Make sure that the controller output settles in the middle of its previous oscillations.

#### 2.3.11 Offset

Feedforward control strategies and direct disturbance rejection are possible with the `PIC_Controller` block via the offset function. If the value of the disturbance variable is measured or estimated, or an external feedforward compensator is utilized, assign the feedforward control setpoint to an [`offset`](https://github.com/controltheplant/PID-controller/blob/main/readme.md#input-parameter) input of the block. 

> [!IMPORTANT]
> The feedforward control setpoint must be scaled to a range from 0 to 100% by external software.

The offset value is influenced by the output rate limit function, anti-windup, and the limit of controller output functions.

#### 2.3.12 Anti-windup

Nonlinearities of the control element caused by its physical limitations can lead to a phenomenon known as integrator windup, which is expressed in the fact that controller output reaches the actuator limits. When the actuator is saturated (i.e. control signal is at the limits of the actuator) and no countermeasures are taken, it may take a long time to return the control signal to normal value. Integrator windup can be caused by such incidents as a failure of control loop equipment, disruption of the technological process, large setpoint step change, or large disturbances. The `PID_Controller` block has three methods to prevent integrator windup:

> [!IMPORTANT]
> Higher and lower limits of the actuator must be specified via `settings.irOutLimitH` and `settings.irOutLimitL` parameters. Their default values are 100% and 0% respectively, which fits most cases. Refer to [Limit](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2313-limit) function.

##### Integrator clamping

This method is a conditional integration. The integrator stops integration when the controller output is saturated, and the control deviation directs the integrator to even more saturation. Make the following assignment to select this method:

```
settings.iwAntiWindupMethod := enumAntiWindupMethods.CLAMPING;
```

##### Back-calculation using the position of the actuator

If the actuator position is measured, it's possible to calculate the actual controller saturation value and subtract it from the integral term of the controller. Controller output will be recalculated and its value will not exceed the limits. The principle of back-calculation is shown in Figure 4.

<p align="center">
    <img src="https://github.com/user-attachments/assets/bbd287cb-4583-4dc1-8490-5d2fd85cf010"  alt="Back calculation diagram"><br>
    Figure 4 - Functional scheme of back-calculation
</p>

here T<sub>b</sub> is a time constant of back calculation. Its value is specified via the `settings.irBackCalcFactor` parameter and should be adjusted empirically. The lower this parameter, the more aggressive the anti-windup action. However, note that too low value of this parameter can lead to instability of the control loop. It's clear that when the actuator is not saturated, the back-calculation value is zero. Make the following assignment to select this method:

```
settings.iwAntiWindupMethod := enumAntiWindupMethods.BACK_CALC_REAL;
```

It is necessary to assign the measured actuator position to the `actuatorPos` input of the block. The actuator position must be scaled to a range from 0 to 100%.

##### Back-calculation using the model of the actuator

If the actuator position is not measured, it can be estimated from the model of the actuator. This model comprises a saturation element and an optional rate limiter. The operating principle of this method is the same as the previous method. The `settings.irBackCalcFactor` parameter should also be adjusted. You can also specify the rate of change of the actuator in the `settings.irOutRampRate` parameter, although this is optional. Make the following assignment to select this method:

```
settings.iwAntiWindupMethod := enumAntiWindupMethods.BACK_CALC_MODEL;
```

The value of the measured actuator position is not needed with this method.

#### 2.3.13 Limit

The limit function allows to constrain the range of the output value to a user-specified range. To specify the range of the controller output value, set the `settings.irOutLimitH` for a high limit and `settings.irOutLimitL` for a low limit. Those values will be taken into account for [Anti-windup](https://github.com/controltheplant/PID-controller/blob/main/readme.md#2312-anti-windup) function.

### 2.4 Referenced objects

| Functions            | Data Types |
|------                |------|
| FindSteadyState (FB / V1.0.0) | typePIDSettings (DUT / V1.0.0) |
| LSim_PT1 (FB)        | enumAntiWindupMethods (ENUM / V1.0.0) |
| RateLimiter (FB)     | enumBumplessTransferMethods (ENUM / V1.0.0) |
| MovingAverage (FB)   | |
| T_PLC_US (FC)        | |
| SCALE_R (FC)         | |
| DeadBand (FC)        | |

### 2.5 Change Log

| Version & Date | Change description |
|-----|-----|
| **01.00.00**<br/> 08.2024 | **controltheplant**<br/> First released version |
| **01.01.00**<br/> 08.2024 | **controltheplant**<br/> - back calculation section changed <br/> - other minor fixes |