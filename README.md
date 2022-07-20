# NASA-COG

## Introduction

This is a repo for a control system made for NASA to drive a Ceramic Oxygen Generation
technology they have developed. It was forked from ["The Ox"]() project of Public Invention run by Mr. Ben Coombs, which was in turn a fork of the VentOS project of Helpful Engineering let my Ben Coombs, Dr. Erich Schulz, and Robert L. Read.

However, the main reason we are starting with this fork is not that the Pressure Swing
Adsorption system of the Ox is a particularly relevant, but rather that the code
represents a nice platform of simple microcontroller control of related machines.

To wit, includes PlatformIO configuration, a state machine (finite automata),
a superloop architecutre, and a hardware abstraction layer.

This particular project of NASA will improve life support technology in space,
but may eventually greatly improve global health.

## How to run

At present, this code does little (for the NASA COG project). We intend
to build a native (that is, laptop) simulation using mock hardware components.
The fundamental way to run this is to perform:
> cd firmware

> pio run -e native && ./.pio/build/native/program

We are currently creating the [RibbonFish](https://github.com/PubInv/NASA-COG/blob/develop/RibbonFish.md) proof-of-concept.

This can be run on a Due by executing:

> make pio-run-due_ribbonfish

As this develops, it will use a specific Due hardware configuration,
please see the [documentation](https://github.com/PubInv/NASA-COG/blob/develop/RibbonFish.md) for details to build your own.


## Annoyances

AFAIK, PlatformIO forces a directory tree depth of 5 levels.

This is not enough to organize the Hardware Abstraction Level they way I would prefer, forcing us to make some messy directories that are less than clear.

For example, to implement a Temperature Sensor, we would ideally have an
abstract (or virtual) implementation, a folder for real hardware,
and a folder for mock sensors.  I have no choice now but to jam
the mock sensors into the same directory with the real hardware and
use a naming convnention.

## The Command System

The NASA-COG system needs to allow the user to enter commands. The most obvious of these is "turn off".
Because the system should avoid thermal shocks, this is actually not simple.

We have previously used an [approach](https://github.com/PubInv/PIRCS-pubinv-respiration-control-standard) of using simple JSON objects to represent commands. We propose to use the same idea here, but obviously,
the command will be custom to the NASA-COG.

### Parameter Setting Commands
The general scheme is to use a JSON object like:

```JavaScript
{ "com": "T",
  "par": 7,
  "mod": "T",
  "val" : 732.3
  }
```

1. "com" is the command. For example, "T" can mean "set Temperature".
2. "par" is a parameter used to specify something about how to execute the command. For example, 7 could mean location 7, which might be between the 6th and 7th stack.
3. "mod" is a modifier of the command. For example, the "set Temperature" command could be used to
set the minimum, maximum, or the target temperature.
4. "val" is the value that is to be set.

For this problem space, we can specify some global parameters of the problem, some of which
involve safety. These can of course have default values "hard wired" into the code, but an
experimenter may wish to change these values (carefully).

1. The maximum rate of voltage change applied to a stack. Nominally we have defined this as 1 volt/second.
We have evidence that too rapid a voltage change (such as that caused by pulse-width modulation) can destroy
a stack.
2. The maximum rate of temperature change for a stack. Nominally we may say this as 2 degree K per minute,
which would allow the machine to reach operating temperature in 6 hours.
3. The maximum temperature difference across a stack in any dimenstion. We may not be able to control
this directly, but we can use it as a target.
4. The maximum voltage applied to a stack.
5. The maximum amperage applied to a stack.

### Event-like Commands

We will need other commands that represent discrete events in time. For example, each transition in the
state transition diagram below may be initiated by a command from the user (they may also occur
automatically due to an internal fault or even normal operation.)

![COG State Machine](https://user-images.githubusercontent.com/5296671/180069835-ce7cdff7-c445-45c1-967d-089810e837db.png)



## License

In general we intend to follow the [Public Invention Licensed Guidelines](https://github.com/PubInv/PubInv-License-Guidelines) unless
NASA requests something different. If you are starting your own project, please
feel free to fork the license guidelines as a starting point if you like our
license policy.

This program includes free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

See the GNU Affero General Public License for more details. You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
