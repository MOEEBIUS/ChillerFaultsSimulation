# Simulation of Common Chiller Faults


## Fault-free Operation (FaultFreeOperation.mo)

This model allows the user to simulate the Fault Free Operation of a Chiller. 


## Reduced Condenser Water Flow Fault (ReducedCondFlow.mo)

If the pump in cooling water loop doesn’t work properly or some pipes are leaking, the condenser water flow will be reduced. 
This fault is emulated by a decreasing ramp input of chilled water mass flow rate, while different severity levels for the 
faults are implemented.


## Reduced Evaporator Water Flow Fault (ReducedEvapFlow.mo)

A malfunctioning pump or leakage of pipes in the chilled water loop can cause reduced evaporator water flow. This fault is emulated 
by a decreasing ramp input of chilled water mass flow rate, while different severity levels for the faults are implemented.


## Cooling Tower Malfunction Fault (CoolingTowerMalfunction.mo)

A cooling tower malfunction will cause insufficient heat dispersion in the cooling water loop, so the cooling water inlet temperature 
will increase. This fault is emulated by increasing the cooling water inlet temperature by a ramp input and 
different severity levels for the fault are implemented.


## AHU Fan Malfunction Fault (AHUFanMalfunction.mo)

If there is a fault in the AHU Fan, reduced or no wind will blow through the coil. This will cause the chilled water to flow through 
the coil and the heat transfer with the air to be insufficient, causing a lower chilled water inlet temperature. 
This fault is simulated by a ramp input which reduces the inlet temperature.


## Dependencies

This model is based on ThermoCycle Library (http://www.thermocycle.net/) and 
ExternalMedia Library (https://github.com/modelica/ExternalMedia) components. 


## License

This model is released by Technische Hochschule Nuernberg Georg Simon Ohm under the Modelica License 2.0.


## Acknowledgments

Parts of this work have been developed with funding from the European Union's Horizon 2020 
research and innovation programme under grant agreement No 680517 (MOEEBIUS)

