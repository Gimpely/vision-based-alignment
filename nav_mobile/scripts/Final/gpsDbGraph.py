
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation

from ublox_msgs.msg import NavSAT
from matplotlib.lines import Line2D

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        langs = ['NOPE']
        students = [10] 
        self.ln = self.ax.bar(langs,students)
        self.gpsDb = [0]
        self.gpsID = [0]
        self.color = ["black"]
        self.usedId = [0]
   

    def plot_init(self):
        #self.ax.set_xlim(0, 10000)
        #self.ax.set_ylim(-7, 7)
        return self.ln


    def callback(self, msg):

        data = NavSAT()
        #print(msg)
        
        self.gpsDb = []
        self.gpsID = []
        self.usedId = []
        self.color = ["black"]
        i = 0

        color=['black', 'red', 'green', 'blue', 'cyan']
        #legendT = 
        colorST = 0
        previousID = 0

        if msg.sv[0].gnssId == 0:
            self.usedId.append(0)
        for gps in msg.sv:

            #print(gps.cno)
            if gps.cno != 0 : 

                if previousID != gps.gnssId:
                    previousID = gps.gnssId
                    colorST = colorST + 1 
                    self.usedId.append(gps.gnssId)
                    if colorST > 4 :
                        colorST = 0


                #print(gps.cno)
                self.gpsDb.append(gps.cno )
                self.gpsID.append(i)
               
                self.color.append(color[colorST])
                i = i + 1


    
    def update_plot(self, frame):
        self.ax.clear()

        try:
            self.ax.hlines([40, 30], -1, self.gpsID[-1]+1, color ="cyan")
        except:
            pass
        
        self.ax.bar(self.gpsID,self.gpsDb, color=self.color,align='center')
        custom_lines = [Line2D([0], [0], color="black", lw=4),
                        Line2D([0], [0], color="red", lw=4),
                        Line2D([0], [0], color="green", lw=4),
                        Line2D([0], [0], color="blue", lw=4),
                        Line2D([0], [0], color="cyan", lw=4)]
        try:
            self.ax.legend(custom_lines, self.usedId )
        except:
            pass
        
        #self.ax.legend(['First line', 'Second line'])

        #self.ln.set_data(self.x_data, self.y_data)
        return self.ln


rospy.init_node('lidar_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/gps/navsat', NavSAT, vis.callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 