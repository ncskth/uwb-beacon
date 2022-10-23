![](docs/swarm.jpg)

# NCS UWB beacon
The beacons can help another node to position itself by transmitting their position and distance. 
A minimum of 4 beacons are needed to uniquely calculate the position in 3D space but more will increase the accuraccy.

## how to use
### usage
A typical beacon system will consist of the following nodes:
* 1x origin-beacon (mandatory)
* 1x X-beacon (mandatory)
* 1x Y-beacon (mandatory)
* 1x Z-beacon (mandatory)
* Nx repeater beacons (optional)

The X, Y, Z and origin-beacons are mandatory and span the coordinate system. The distance at which they are placed is not important other than for a few best practices. Repeater beacons are optional and only exist to improve the performance. All beacons must be assigned an ID that is globally unique over the WHOLE uwb system, so not just beacons but i.e. Durin robots as well.

A fourth mode, passive, is also available. Intended to be used on for example moving objects. The beacon will become a part of the positioning system but it will still report its distance to whoever polls it.

### calibration
The beacons are almost entirely self calibrating. The only thing that needs special care is ensuring that all axises are in the proper order and orthogonal. To calibrate the system press the small button and you should see all beacons turn yellow. Then you have a few seconds to get out of the way. The beacons really hate having a big human filled with water obstructing their view.

If everything is in order then one beacon will show white which means that it is currently being calibrated. Each beacon takes in the order of 10 seconds to calibrate.

Once all beacons are calibrated they will all show green.

If the beacons turn pink after a few seconds the calibration has failed. Most likely because one of the 4 mandatory beacons are missing.

### configuration
The beacons have a ft232 connected to the USB port which is used for configuration. The UART paramters are the following (usually the defaults for almost all setups I have used).

* 115200 baud
* 8 data bits
* no parity bit
* 1 stop bit

It runs a normal console and some of the things you can configure are.
* node id
* beacon mode
* wifi on/off (should only be on for debugging)

run `help` for exact details

### best practices 


## developer guide
### overview

### messages

### usage

### protocol