import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

import csv

particle_data = {}
trajectory_data = {}

def main():
    # key is timestamp


    with open("particles.csv") as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            t = float(row[0])
            s = float(row[1])
            v = float(row[2])
            m = int(row[3])
            dist = int(row[4])
            if t not in particle_data:
                particle_data[t] = {}
            if dist not in particle_data[t]:
                particle_data[t][dist] = {}
            if m not in particle_data[t][dist]:
                particle_data[t][dist][m] = []
            particle_data[t][dist][m].append((s,v))

    # ego_s = []
    # ego_v = []
    # obj_s = []
    # obj_v = []
    # with open("trajectories.csv") as csvfile:
    #     reader = csv.reader(csvfile)
    #     for row in reader:
    #         ego_s.append(float(row[0]))
    #         ego_v.append(float(row[1]))
    #         obj_s.append(float(row[2]))
    #         obj_v.append(float(row[3]))


    fig, ax = plt.subplots()
    plt.subplots_adjust(left=0.25, bottom=0.25)

    plt.xlim(-30, 200)
    plt.ylim(0,35)

    distToPlot = 1 # intermediate
    scat0 = plt.scatter([el[0] for el in particle_data[1.5][distToPlot][0]],
                        [el[1] for el in particle_data[1.5][distToPlot][0]],
                        s=3.0)
    scat1 = plt.scatter([el[0] for el in particle_data[1.5][distToPlot][1]],
                        [el[1] for el in particle_data[1.5][distToPlot][1]],
                        s=3.0)
    scat2 = plt.scatter([el[0] for el in particle_data[1.5][distToPlot][2]],
                        [el[1] for el in particle_data[1.5][distToPlot][2]],
                        s=3.0)

    # these plots are fixed and wont move with slider
    # these don't quite look right.
    # plt.plot(ego_s, ego_v)
    # plt.plot(obj_s, obj_v)

    ax.margins(x=0)

    t0 = 1.5
    delta_t = 0.5

    axcolor = 'lightgoldenrodyellow'
    axtime = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)

    time = Slider(axtime, 'timestep', 1.5, 10.0, valinit=t0, valstep=delta_t)

    def update(time):
        print(time)
        x = [el[0] for el in particle_data[time][distToPlot][0]]
        y = [el[1] for el in particle_data[time][distToPlot][0]]
        xx = np.vstack((x,y))
        scat0.set_offsets(xx.T)

        x = [el[0] for el in particle_data[time][distToPlot][1]]
        y = [el[1] for el in particle_data[time][distToPlot][1]]
        xx = np.vstack((x,y))
        scat1.set_offsets(xx.T)

        x = [el[0] for el in particle_data[time][distToPlot][2]]
        y = [el[1] for el in particle_data[time][distToPlot][2]]
        xx = np.vstack((x,y))
        scat2.set_offsets(xx.T)


        fig.canvas.draw_idle()


    time.on_changed(update)

    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
