import math;

def fmt(value):
    return "%.3f" % value

period = 4
radius = 0.5
timestep = 0.05
maxtime = period*1
data_x = 0
data_y = 0
x0 = 0
y0 = 0
v_x = 0
v_y = 0


with open('SpiralFF.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius;
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + "-1\n");
        t += timestep;
        radius += 0.01 

radius = 0.5

with open('SpiralNoFF.txt', 'w') as the_file:
    t=0;
    while t <= maxtime:
        x = math.sin(t * 2 * math.pi / period) * radius;
        y = math.cos(t * 2 * math.pi / period) * radius;
        if (t > 0):
            v_x = (x - x0) / timestep
            v_y = (y - y0) / timestep
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + "-1" + "," + fmt(v_x) + "," + fmt(v_y) + "," + "-1\n");
        t += timestep;
        x0 = x
        y0 = y
        radius += 0.01    
