import time
import os
import sys
import struct
myfolder = os.path.dirname(os.path.realpath(__file__))

def gen_triangular_speed_profile():
    speed = 300
    for i in range(0,180):
        speed = 300 + i
        yield "G1 X0.25 F%f"%(speed)
    for i in range(0,180):
        slowdownspeed = speed - i
        yield "G1 X0.25 F%f"%(slowdownspeed)

def gen_jumping_speed_profile():
    speed = 400
    distance = 0.2
    for i in range(0,20):
        yield "G1 X%f F%f"%(distance, speed)
        #yield "G1 X%f F%f"%(distance/10, speed/10)
        #yield "G1 X%f F%f"%(-distance/10, speed/10)
        yield "G1 X%f F%f"%(-distance, speed)
        yield "G1 X%f F%f"%(-distance*5, speed*4)
        yield "G1 X%f F%f"%(-distance*5, speed*4)

pipe = '/tmp/printer'
f = open(pipe,"w+")
with f:
    f.write("G91\n")
    for gc in gen_jumping_speed_profile():
        print gc
        f.write(gc )
        f.write("\n")
        

