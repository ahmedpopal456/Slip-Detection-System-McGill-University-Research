# Slip_Detection_System_nrf52_MCU

Multi-MCU Project, with the goal of collecting Analog and Digital Data from both heels and the pelvis, all while storing it
into an SD card. This is achieved by having three RF-compatible custom designed-PCBs' attached to both heels and the pelvis.

The pelvis schedules the data of both heels through RF, and, with the correct timestamps, stores the value of all seven pressure 
sensors (located under the foot, in an insole) and of all three acceleration axis' (relative to the heel)

While doing so, it also individually collects acceleration data along the pelvis. It then stores all the acquired data in a .CSV
file at 200 Hz, inside an SD card (which is to be injected inside the pelvis PCB). 

The .CSV file is then fed onto a MATLAB GUI, that will parse the data, draw all of the acceleration and pressure graphs/maps, 
and through a heuristic based algorithm, will determine when a slip occured during the time the user was wearing the device.

This information will be used by researchers at McGill University to improve the condition of construction workers, 
especially on the decision behind choosing the correct footwear for them. 
