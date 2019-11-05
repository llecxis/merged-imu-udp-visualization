# from PyQt5.QtWidgets import QMainWindow, QFileSystemModel, QTreeView, qApp, QApplication, QDockWidget,QAction, QHBoxLayout, QVBoxLayout, QPushButton, QWidget, QListWidget, QToolBar, QTextEdit, QLabel, QGridLayout
import sys
# from PyQt5.QtGui import QIcon
from PyQt5 import QtGui
# from PyQt5.QtCore import Qt, QDir
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import vtk
import math
import numpy as np
import time
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import socket
import threads_qt
import traceback

# self.label_status = QLabel('', self)         layout.addWidget(self.label_status)            self.label_status.setText(status)
def trap_exc_during_debug(*args):
    # when app raises uncaught exception, print info
    print(args)

# install exception hook: without this, uncaught exception would cause application to exit
sys.excepthook = trap_exc_during_debug

class vtpDrawScene: 
    def SetQuatOrientation( self, quaternion, shift, i_actor ):
        # if not self.iniOk :
        #     raise Exception("vtpDrawScene not initialized. Call initScene() first")

        # Convert quat to the rotation matrix
        # self.mtxRot =  [[0,0,0],[0,0,0],[0,0,0]]
        vtk.vtkMath().QuaternionToMatrix3x3(quaternion, self.mtxRot[i_actor])
        # print(self.mtxRot[i_actor])

        # norm matrix
        self.mtxRot[i_actor] = np.array(self.mtxRot[i_actor]).dot(np.array(self.norm_mat[i_actor]))

        # Rotation: convert 3x3 to 4x4 matrix
        mtxTr2 = vtk.vtkMatrix4x4() # identity mtx
        for i in range(3):
            for j in range(3) :
                mtxTr2.SetElement(i, j, self.mtxRot[i_actor][i][j])
      
        # three transforms:
        # 1. move the object so the rotation center is in the coord center  
        tr = vtk.vtkTransform()
        origin = np.array(self.modelActor[i_actor].GetOrigin())
        
        position = np.array(self.modelActor[i_actor].GetPosition())
        #trans = origin + position
        trans = position
        #trans = origin
        tr.Translate(-trans)
        mtxTr1 = tr.GetMatrix()
        
        # 2. rotate around coord center using mtxTr2
        mtxTr12 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr2, mtxTr1, mtxTr12)
        
        ## 3. move the object back
        tr = vtk.vtkTransform()
        tr.Translate(trans + np.array(shift))
        mtxTr3 = tr.GetMatrix()
        mtxTr123 = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4().Multiply4x4 (mtxTr3, mtxTr12, mtxTr123)
        
        tr = vtk.vtkTransform()
        tr.PreMultiply()  
    #    tr.PostMultiply()  
        tr.Concatenate(mtxTr123) 
        self.modelActor[i_actor].SetUserTransform(tr)

        # self.ren.Render()
        # return self.ren
        # self.renWin.Render()
    
    def initScene_qt(self, obj):
        reader = list()
        modelMapper = list()
        self.modelActor = list()
        self.mtxRot = list()
        self.norm_mat = list()

        self.ren = vtk.vtkRenderer()

        for i in range(0,len(obj)):

            # Read the object data
            filename = obj[i]
            reader.append(vtk.vtkXMLPolyDataReader())
            reader[i].SetFileName(filename)
            reader[i].Update()
            
            # make mapper for the data
            modelMapper.append(vtk.vtkPolyDataMapper())
            modelMapper[i].SetInputData(reader[i].GetOutput())
            
            # create actor, set its mapper
            self.modelActor.append(vtk.vtkActor())
            
            self.modelActor[i].SetMapper(modelMapper[i])
            
            # Add the actor to the renderer, set the background and size.
            self.ren.AddActor(self.modelActor[i])
            
            # 
            self.mtxRot.append([[0,0,0],[0,0,0],[0,0,0]])
            # Get center of the bounding box           
            
            origin = self.modelActor[i].GetCenter()
            #
            #  Set rotation center
            self.modelActor[i].SetOrigin(origin)
             
        #Initial conditions for bones
        self.initial_pos_actors = [[0,0,0],[0,0,0],[0,0,0],[-0.016, -0.006, 0.163],[-0.016, -0.006, -0.163],[-0.01,  -0.312, -0.173],[-0.01,  -0.312, 0.173],[-0.01, 0.024, 0.166],[-0.01, 0.024, -0.166],[0,0,0],[-0.005, -0.296, -0.153],[-0.005, -0.296,  0.153]]
        for el in range(len(self.initial_pos_actors)):
            self.modelActor[el].SetPosition(self.initial_pos_actors[el])
            self.norm_mat.append([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  #[[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]])

        # self.modelActor[3].SetPosition(-0.016, -0.006, 0.163)
        # self.modelActor[4].SetPosition(-0.016, -0.006, -0.163)
        # self.modelActor[7].SetPosition(-0.01, 0.024, 0.166)
        # self.modelActor[8].SetPosition(-0.01, 0.024, -0.166)

        # self.modelActor[5].SetPosition(-0.01,  -0.308, -0.173)
        # self.modelActor[6].SetPosition(-0.01,  -0.308, 0.173)
        # self.modelActor[11].SetPosition(-0.005, -0.296,  0.153)
        # self.modelActor[10].SetPosition(-0.005, -0.296, -0.153)


        axes = vtk.vtkAxesActor()
        axes.SetNormalizedTipLength(0.05, 0.05, 0.05)
        axes.SetNormalizedShaftLength(1,1,1)
        self.ren.AddActor(axes)
        #  The axes are positioned with a user transform
        transform = vtk.vtkTransform()
        transform.Translate(0.0, 0.0, 0.0)
        axes.SetUserTransform(transform)

        self.ren.SetBackground(0.1, 0.2, 0.4)

        camera = vtk.vtkCamera()
        camera.SetPosition(2, 0.2, 2)
        camera.SetFocalPoint(0, 0, 0)
        self.ren.SetActiveCamera(camera)
  
        # SetInteractorStyle(MyInteractorStyle())
        #ren.ResetCamera()
        # ren.Render()
        self.iniOk_qt = True
        return self.ren

    # def inter(self, flag):
    #     if (flag == 0):
    #         interactor = vtk.vtkRenderWindowInteractor()
    #         interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())        
    #         interactor.AddObserver("KeyPressEvent",KeyPress)
    #         interactor.SetRenderWindow(self.renWin)
    #         interactor.Start()
    #     if (flag == 1):
    #         interactor.ReInitialize()
    #         interactor.Start()            
    #     if (flag == -1):
    #         interactor = None
    #     return

    def __init__(self):
        self.iniOk = False

    # def __del__(self):
    #     self.ren.Finalize()
    
    def init_pos_actor( self, i_actor, mov):
        # if not self.iniOk :
        #     raise Exception("vtpDrawScene not initialized. Call initScene() first")
        # set initial pos for hands
        real_pos = np.array(self.modelActor[i_actor].GetPosition())
        new_pos = real_pos + mov
        print(new_pos)
        self.modelActor[i_actor].SetPosition(new_pos)
        
        # self.renWin.Render()



class Worker(QObject):
    """
    Must derive from QObject in order to emit signals, connect slots to other signals, and operate in a QThread.
    """

    sig_shifts = pyqtSignal(int, list)  # worker id, step description: emitted every 5 steps through work() loop
    sig_qtr = pyqtSignal(int, list)  # worker id, step description: emitted every step through work() loop
    sig_status = pyqtSignal(int, int)  # worker id: emitted status()
    sig_msg = pyqtSignal(str)  # message to be shown to user

    

    def __init__(self, id: int, port: int):
        super().__init__()
        self.__id = id
        self.port = port
        self.__abort = False

    @pyqtSlot()
    def work(self):

        import sys
        import socket, traceback, re, math, threading, time, random, numpy
        import skinematics as skin
        import matplotlib.pyplot as plt
        import select
        import numpy as np
        import vector
        from scipy import interpolate
        from scipy.signal import savgol_filter
        te = 100
        
        emptylist = list()

        """
        Pretend this worker method does work that takes a long time. During this time, the thread's
        event loop is blocked, except if the application processEvents() is called: this gives every
        thread (incl. main) a chance to process events, which in this sample means processing signals
        received from GUI (such as abort).
        """
        thread_name = QThread.currentThread().objectName()
        thread_id = int(QThread.currentThreadId())  # cast to int() is necessary
        self.sig_msg.emit('Running worker #{} from thread "{}" (#{})'.format(self.__id, thread_name, thread_id))

        # for step in range(100):
        #     time.sleep(0.1)
        ##################################################
        host = ''

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind((host, self.port))

        beg = time.time()
        flag = 1
        x = list()
        times = list()
        linacc = list()
        rotmat = list()
        xnew = list()
        mean_vel = list()
        integrated_lin_acc = list()
        e = np.matrix([[1.,0.,0.],[0.,1.,0],[0.,0.,1.]])
        i = 0
        shifts = [0.,0.,0.]
        times.append(0.)

        while 1:
            try:                
                ready = select.select([s], [], [], 1)
                if (ready[0] == []):
                    self.sig_status.emit(self.__id, 0)

                message, address = s.recvfrom(8192)
                sig_time = times[-1]             
                            

                d = parse(message) #time and dictionary
                
                if flag: #start time flag
                    st = float(d['time'])
                    

                d['time'] = [ round(float(d['time']) - st,6)]
                d['linacc'] = [float(item) for item in d['linacc']]

                #emptylist.append(d)

                diff = time.time() - beg

                #emptylist.append(', '.join(el for el in numpy.concatenate([d['time'],d['acc'],d['gyr'],d['mag'],d['grav'], d['linacc'],d['rotvec'][0:4],d['rotmat'][0:3],d['rotmat'][4:7],d['rotmat'][8:11]])))

                linacc.append(d['linacc'])
                if flag: #start time flag
                    times[0] = d['time']
                    flag = 0
                else:
                    times.append(d['time'])

                qtr = [float(item) for item in d['rotvec'][0:4]]

                # qtr = [float(d['rotvec'][2]),float(d['rotvec'][0]),float(d['rotvec'][1]),float(d['rotvec'][3])]


                #print(qtr)
                self.sig_qtr.emit(self.__id,qtr)

                i += 1
                lenth = 25 #длинна окна усреденения
                dis = 5 #5 #изменение траектории каждые 5 знчений
                self.sig_shifts.emit(self.__id, shifts)
                #print(len(times), " ", i)

                ############################################################################################################       calculating shifts           
            #     if (i % dis == 0) and (i > 25): #(i % dis == 0) or (i > 25): #(i == 199): # (i == 990): (i == int(te*100-2))
            #         # print(linacc)
            #         accnp = np.array(linacc)
            #         x = np.array(times)                
            #         #print(times)
            #         # y0 = accnp[:,0]

            #         y0 = savgol_filter(accnp[:,0], lenth, 3)
            #         y1 = savgol_filter(accnp[:,1], lenth, 3)
            #         y2 = savgol_filter(accnp[:,2], lenth, 3)

            #         # print(len(x))
            #         # print(len(y0))
            #         # print(y0)

            #         tck0 = interpolate.splrep(x, y0, s=0)
            #         tck1 = interpolate.splrep(x, y1, s=0)
            #         tck2 = interpolate.splrep(x, y2, s=0)
                                
            #         ynew = interpolate.splev(x, tck0, der=0)

            #         mean_vel.append(np.mean(ynew[-dis:]))
            #         x_mean_vel = np.linspace(0,5, num = len(mean_vel))

            #         #print(ynew)
            #         #print(y0)  

            #         #print(len(x), ' ', abs(int(np.mean(ynew[-dis:]) * 1000)))


            #         # check = np.mean(ynew[-dis:]) * 1000.
            #         # if not(np.isnan(check)):
            #         #     if (abs(int(check)) > 10 ):

            #         yint0 = integ(x, tck0)
            #         yint1 = integ(x, tck1)
            #         yint2 = integ(x, tck2)
                    
            #         tck00 = interpolate.splrep(x, yint0, s=0)
            #         tck10 = interpolate.splrep(x, yint1, s=0)
            #         tck20 = interpolate.splrep(x, yint2, s=0)

            #         yint00 = integ(x, tck00)
            #         yint10 = integ(x, tck10)
            #         yint20 = integ(x, tck20)

            #         shifts = [yint20[-1,0],yint10[-1,0],yint00[-1,0]]
            #         shifts = [0.,0.,0.]
            #         self.sig_shifts.emit(self.__id, shifts)

            #         ##################################################################################################################
                                            
            #             # else:
            #             #     for j in range(dis): # корректировка траектории если скорость была мала
                                
            #             #         linacc[-1-j] = [0.0,0.0,0.0]

            #     # if (i > 498):
            #     #     plt.figure()
            #     #     plt.plot(x, ynew, x, yint0, x, yint00, '--', x_mean_vel, mean_vel ) #yint00, yint0, y0 #, x, yint0, x, yint00, '--'
            #     #     plt.legend(['data','velocity', 'trajectory'])
            #     #     plt.axis([0.0, 3., -2, 2])
            #     #     plt.title('Integral estimation from spline')
            #     #     plt.show()

                if (1) and (sig_time != times[-1]): #shifts != [0.,0.,0.]
                    self.sig_status.emit(self.__id, 1)
                    #print(times[-1],times[-2])

            #     # times.append(d['time'])
            #     # linacc.append([float(item) for item in d['linacc'] ])
            #     # rotmat.append( [[float(item) for item in d['rotmat'][0:3]], [float(item) for item in d['rotmat'][4:7]], [float(item) for item in d['rotmat'][8:11]]] )

            #         # check if we need to abort the loop; need to process events to receive signals;
            #     App.processEvents()  # this could cause change to self.__abort
            #     if self.__abort:
            #         # note that "step" value will not necessarily be same for every thread
            #         self.sig_msg.emit('Worker #{} aborting work at step {}'.format(self.__id, step))
            #         break

                # if diff < te:
                #     pass
                # else:
                #     # with open('test_data/' + str(port)+ str(time.strftime("_%d_%m_%Y_%H_%M_%S",time.gmtime(time.time()))) + '.csv', 'a') as the_file:
                #     #     the_file.write('\n'.join(el for el in emptylist))
                #     print(str(port), " - stoped" )               
                #     break

            
    
            except (KeyboardInterrupt, SystemExit):
                raise
            except:
                traceback.print_exc()
        
        ##################################################

    def abort(self):
        self.sig_msg.emit('Worker #{} notified to abort'.format(self.__id))
        self.__abort = True


def get_files(directory):
    import os
    files = os.listdir(directory)
    for i in range(0,len(files)):
        files[i] = directory + files[i]
    return files

def parse(mes):
    mes = mes.decode("utf-8").replace(" ", "")
    tm = mes.split("#")[0]
    d = dict([(el.split(",")[0], el.split(",")[1:]) for el in mes.split("#")[1:]])
    d["time"] = tm
    
    return d

def integ(x, tck, constant = 10e-9):
    from scipy import interpolate
    x = np.atleast_1d(x)
    out = np.zeros(x.shape, dtype=x.dtype)
    for n in range(len(out)):
        out[n] = interpolate.splint(0, x[n], tck)
    out += constant
    return out

def swap(self, i, j):
    s = self.a[i]    
    self.a[i] = self.a[j]
    self.a[j] = s

def NextSet(self, n):                #операция перестановки
    
    j = n-2
    while ( (j!= -1) and (self.a[j] >= self.a[j+1]) ):
        j-=1

    if (j == -1):
        return False
    k = n - 1
    while (self.a[j] >= self.a[k]):
        k-=1    
    swap(self, j, k)   

    l = j + 1
    r = n - 1
    while (l < r):
        swap(self, l , r)
        l += 1
        r -= 1
    return True

# 3,1,2,4

def cicle(self, data):
    return [data[self.a[0]], data[self.a[1]], data[self.a[2]], data[self.a[3]] ]

class MainWindow(QMainWindow):

    NUM_THREADS = 2 # number of phones
    sig_abort_workers = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.title = "Система сбора, обработки и визуализации деятельности работников предприятия"
        self.top = 200
        self.left = 500
        self.width = 1000
        self.height = 800
        
        self.setWindowIcon(QtGui.QIcon("icon.png"))
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        
        self.createMenu()
        self.createSensorsDock()  

        self.createCentralWidget()
        self.createToolBar()

        self.createExplorerDock()
        
        self.createLogDockWidget()

        self.configureClicks()

        self.log_text.append("Initialization Ok")

        self.show()
    

    def clearLog(self):
        # command = self.log_text.toPlainText()
        self.log_text.clear()
        # self.log_text.setText(str(eval(command)))

    def configureClicks(self):
        self.clear_log.clicked.connect(self.clearLog)
        self.play.clicked.connect(self.vtkCall)
        self.stop.clicked.connect(self.vtkEndCall)
        self.tuning_n.clicked.connect(self.n_qtr_shift)
        self.tuning_z.clicked.connect(self.z_qtr_shift)
        self.tuning_t.clicked.connect(self.t_qtr_shift)
        
        # self.new_ren.connect(self.vtkEndCall)

        # прописывать обработку всех кликов по кнопкам

    def n_qtr_shift(self): #нужно вставаить соответствие между масивами актеров и потоками
        # temp_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])
        # sol_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])
        self.log_text.append("N - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]))
        self.n_pos_temp_mat = self.scene.mtxRot[4]
        self.n_pos_temp_qtr = self.qtrs[1]
        self.log_text.append(str(self.n_pos_temp_mat))

        # self.log_text.append(self.n_pos_temp_qtr) 

        # temp_n_mat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        
        # sol_mat = np.linalg.solve(temp_mat,temp_n_mat)

        # self.scene.norm_mat[4] = sol_mat

        # self.log_text.append( str(temp_mat) )
        # self.log_text.append( str(sol_mat) )
        # self.log_text.append( str(self.scene.norm_mat[4]) )
        # self.log_text.append( str(np.array(self.scene.mtxRot[4]).dot(np.array(self.scene.norm_mat[4]))) ) 

    def t_qtr_shift(self):
        self.log_text.append("T - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]))
        self.t_pos_temp_mat = self.scene.mtxRot[4]
        self.t_pos_temp_qtr = self.qtrs[1]
        self.log_text.append(str(self.t_pos_temp_mat))

    def z_qtr_shift(self):
        self.log_text.append("Z - pose initialized with quaternions = " + str(self.qtrs[0]) + " " + str(self.qtrs[1]))
        self.z_pos_temp_mat = self.scene.mtxRot[4]
        self.z_pos_temp_qtr = self.qtrs[1]
        self.log_text.append(str(self.z_pos_temp_mat))

    def vtkCall(self):      
        self.play.setDisabled(True)
        self.stop.setEnabled(True)
        self.timer.start(self.timeStep)

    def vtkEndCall(self):
        self.stop.setDisabled(True)
        self.play.setEnabled(True)
        self.timer.stop()

    def KeyPress(self,obj, event):
        import re
        key = obj.GetKeySym() #works fine
        global k
        
        if ( re.match(r'\d', str(key) ) ):
            k = int(key) + 3
            self.log_text.append(self.obj_list[k])
        if (key == "Left"):
            self.scene.init_pos_actor (k,np.array([0.000,0.000,-0.001]))
            self.iren.Render()
        if (key == "Right"):
            self.scene.init_pos_actor (k,np.array([0.000,0.000,0.001]))
            self.iren.Render()
        if (key == "Up"):
            self.scene.init_pos_actor (k,np.array([0.001,0.000,0.000]))
            self.iren.Render()
        if (key == "Down"):
            self.scene.init_pos_actor (k,np.array([-0.001,0.000,0.000]))
            self.iren.Render()
        if (key == "a"):
            self.scene.init_pos_actor (k,np.array([0.000,0.001,0.000]))
            self.iren.Render()
        if (key == "d"):
            self.scene.init_pos_actor (k,np.array([0.000,-0.001,0.000]))
            self.iren.Render()
        if (key == "n"):
                   
            while (NextSet(self, 4)):
                return 0
        return 0
        
    def createCentralWidget(self):

        self.frame = QFrame()
        self.vl = QHBoxLayout()
        
        self.vtkWidget = QVTKRenderWindowInteractor(self.frame)
        
        self.visualWidget = QWidget(self)
        self.play = QPushButton("play", self.visualWidget)
        self.stop = QPushButton("stop", self.visualWidget)
        self.pause = QPushButton("pause", self.visualWidget)
        self.tuning_n = QPushButton("N - pose", self.visualWidget)
        self.tuning_z = QPushButton("Z - pose", self.visualWidget)
        self.tuning_t = QPushButton("T - pose", self.visualWidget)

        self.vl.addWidget(self.vtkWidget)
       
        buttons_layout = QVBoxLayout()
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.play)
        buttons_layout.addWidget(self.stop)
        buttons_layout.addWidget(self.pause)
        buttons_layout.addStretch(1)
        buttons_layout.addWidget(self.tuning_n)
        buttons_layout.addWidget(self.tuning_z)
        buttons_layout.addWidget(self.tuning_t)
        buttons_layout.addStretch(1)

        #Create
        self.scene = vtpDrawScene()
        directory = 'geometry/'
        obj = get_files(directory)
        self.obj_list = obj
        self.ren = self.scene.initScene_qt(obj)

        #Settings
        self.ren.SetBackground(0.2, 0.2, 0.2)
        self.timeStep = 20 #ms
        self.total_t = 0

        #check for phones
        self.start_threads()

        # self.controller = threads_qt.Controller()
        # self.controller.app = QCoreApplication([])       #я вообще не понимаю зачем это тут

        self.renWin = self.vtkWidget.GetRenderWindow()
        self.renWin.AddRenderer(self.ren)
        

        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()
        self.iren.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        self.iren.AddObserver("KeyPressEvent", self.KeyPress)
        
        self.visualWidget.setLayout(self.vl)
        self.visualWidget.layout().addLayout(buttons_layout)


        self.frame.setLayout(self.vl)
        self.setCentralWidget(self.frame)
        # self.show()
        self.iren.Initialize()

        # Create Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.timerCallback)
        # self.timer.start(self.timeStep)
    
    def timerCallback(self):
        #сюда преобразование координат qweqrty
        
        i_actor = 4
        self.scene.SetQuatOrientation(self.qtrs[1], self.shifts[1], i_actor)


        i_actor = 5
        #self.actors_shift = np.array(self.scene.initial_pos_actors[4]) + np.array(self.scene.mtxRot).dot(np.array(self.scene.initial_pos_actors[5]) - np.array(self.scene.initial_pos_actors[4]))
        self.shifts[0] = self.shift_calculus(4,i_actor)
        self.temp_qtr = self.qtr_calculus(4,i_actor)
        # print(self.actors_shift,self.scene.initial_pos_actors[4])
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[0], i_actor)

        i_actor = 10
        self.shifts[0] = self.shift_calculus(4,i_actor)
        self.temp_qtr = self.qtr_calculus(4,i_actor)
        self.scene.SetQuatOrientation(self.temp_qtr, self.shifts[0], i_actor)

        self.iren.Render() #NOT: self.ren.Render()
    
    def shift_calculus(self,i_from_actor,i_to_actor):
        return np.array(self.scene.initial_pos_actors[i_from_actor]) + np.array(self.scene.mtxRot[i_from_actor]).dot(np.array(self.scene.initial_pos_actors[i_to_actor]) - np.array(self.scene.initial_pos_actors[i_from_actor]))- np.array(self.scene.initial_pos_actors[i_to_actor])
    
    def qtr_calculus(self,i_from_actor,i_to_actor): #в последствии будте таблица соответствия актеров и потоков, пока остается константами        
        # print(vtk.vtkQuaterniond(self.qtrs[1]).ToMatrix3x3([[0,0,0],[0,0,0],[0,0,0]]))

        i_from_actor = 1 #в последствии будте таблица соответствия актеров и потоков, пока остается константами      
        i_to_actor = 0
        
        mod_qtr = self.qtrs[1][0]*self.qtrs[1][0] + self.qtrs[1][1]*self.qtrs[1][1] + self.qtrs[1][2]*self.qtrs[1][2] + self.qtrs[1][3]*self.qtrs[1][3] # mod for multiplication of quaternions
        
        # self.qtrs[i_from_actor] - a - quater
        # self.qtrs[i_to_actor]   - b - quater

        a1 = self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][3]
        a2 = self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][1] - self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][3]
        a3 = self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][0] + self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][2] - self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][3]
        a4 = self.qtrs[i_from_actor][3]*self.qtrs[i_to_actor][0] - self.qtrs[i_from_actor][2]*self.qtrs[i_to_actor][1] + self.qtrs[i_from_actor][1]*self.qtrs[i_to_actor][2] + self.qtrs[i_from_actor][0]*self.qtrs[i_to_actor][3]
        
        qtr_multiplication = np.array([a1/mod_qtr,a2/mod_qtr,a3/mod_qtr,a4/mod_qtr])
        # print(qtr_multiplication)
        return np.array(qtr_multiplication) #vtk.vtkQuaterniond(np.array(self.qtrs[1])+np.array(qtr)).Normalized()
    
    def start_threads(self):

        self.__workers_done = 0
        self.__threads = []
        self.__threadsstatus = []
        self.stat = []
        self.qtrs = []
        self.shifts = []
        self.a = [0,1,2,3]
        port = 5555
        for idx in range(self.NUM_THREADS):
            worker = Worker(idx, port)
            thread = QThread()
            thread.setObjectName('thread_' + str(idx))
            self.__threads.append((thread, worker))  # need to store worker too otherwise will be gc'd
            self.__threadsstatus.append(self.sensor_im_1) #нужно продумать как именно добавлять статусы многопоточности в виджите
            self.stat.append(-1)
            self.qtrs.append([1.,0.,0.,0.])
            self.shifts.append([0.,0.,0.])
            worker.moveToThread(thread)

            worker.sig_shifts.connect(self.on_worker_shifts)
            worker.sig_qtr.connect(self.on_worker_qtr)
            worker.sig_status.connect(self.on_worker_status)
            # worker.sig_msg.connect(self.log_text.append)

            # control worker:
            self.sig_abort_workers.connect(worker.abort)

            # get read to start worker:
            # self.sig_start.connect(worker.work)  # needed due to PyCharm debugger bug (!); comment out next line
            thread.started.connect(worker.work) #(self.port)
            port = port + 1
            thread.start()  # this will emit 'started' and start thread event loop
    
    @pyqtSlot(int, list)
    def on_worker_shifts(self, worker_id: int, data: list):
        # self.log_text.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        # print(data)
        try: 
            self.shifts[worker_id] = data
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()
    
    @pyqtSlot(int, list)
    def on_worker_qtr(self, worker_id: int, data: list):
        # self.log.append('Worker #{}: {}'.format(worker_id, data))
        # self.progress.append('{}: {}'.format(worker_id, data))
        #print(data)
        try: 
            self.qtrs[worker_id] = cicle(self,data)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()

    @pyqtSlot(int, int)
    def on_worker_status(self, worker_id: int, flag: int):
        # self.log.append('worker #{} done'.format(worker_id))
        # self.progress.append('-- Worker {} DONE'.format(worker_id))
        if (self.stat[worker_id] == flag):
            pass
        elif (flag == 1):
            self.stat[worker_id] = flag
            # self.__threadsstatus[worker_id].setText("Active")
            self.sensor_im_1.setText('Active')
            self.log_text.append('Phone ' + str(worker_id) + ' is Active')
        else:
            self.stat[worker_id] = flag
            self.sensor_im_1.setText('Waiting')
            self.log_text.append('Connect phone # ' + str(worker_id) + '. Waiting...')
            self.qtrs[worker_id] = [0.95,0.,0.25,0.] #[0.,0.,1.,0.]#

        self.__workers_done += 1
        # if self.__workers_done == self.NUM_THREADS:
            # self.log.append('No more workers active')
            # self.__threads = None

    @pyqtSlot()
    def abort_workers(self):
        self.sig_abort_workers.emit()
        # self.log.append('Asking each worker to abort')
        for thread, worker in self.__threads:  # note nice unpacking by Python, avoids indexing
            thread.quit()  # this will quit **as soon as thread event loop unblocks**
            thread.wait()  # <- so you need to wait for it to *actually* quit

        # even though threads have exited, there may still be messages on the main thread's
        # queue (messages that threads emitted before the abort):
        # self.log.append('All threads exited')


    def exit():
        qApp.quit()

    def restoreWindows(self):
        self.explorer.close()
        self.sensors.close()
        self.log.close()
        # сохранить текст из лога и записать его обратно
        self.createExplorerDock()
        self.createSensorsDock()
        self.createLogDockWidget()

    def menuActionConnect(self, name, func):
        act = QAction(name, self)
        # можно добавить иконки QIcon..
        act.triggered.connect(func)
        return act

    def createMenu(self):
        menubar = self.menuBar()
        file = menubar.addMenu("Файл")
        file.addAction("Создать новый эксперимент")
        file.addAction("Загрузить существующий эксперимент")
        file.addAction(self.menuActionConnect("&Выйти из приложения", exit))

        window = menubar.addMenu("Окна")
        
        window.addAction(self.menuActionConnect("Восстановить окна по умолчанию", self.restoreWindows))

    def createSensorsDock(self):
        self.sensors = QDockWidget("Состояние датчиков", self)
        self.sensors.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        sensorsWidget = QWidget(self)

        self.sensor_im_1 = QLabel(sensorsWidget)
        self.sensor_im_1.setText("Image_1")
        self.sensor_im_2 = QLabel(sensorsWidget)
        self.sensor_im_2.setText("Image_2")
        self.sensor_im_3 = QLabel(sensorsWidget)
        self.sensor_im_3.setText("Image_3")

        sensorsHBox = QHBoxLayout()
        sensorsHBox.addWidget(self.sensor_im_1)
        sensorsHBox.addWidget(self.sensor_im_2)
        sensorsHBox.addWidget(self.sensor_im_3)

        namesHBox = QHBoxLayout()
        namesHBox.addWidget(QLabel("phone_1", sensorsWidget))
        namesHBox.addWidget(QLabel("phone_2", sensorsWidget))
        namesHBox.addWidget(QLabel("phone_3", sensorsWidget))

        vBox = QVBoxLayout()
        vBox.addLayout(sensorsHBox)
        vBox.addLayout(namesHBox)

        sensorsWidget.setLayout(vBox)
        
        self.sensors.setWidget(sensorsWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.sensors)

    def setDirectoryToExplorer(self, directory):
        model = QFileSystemModel()
        model.setRootPath(directory)
        model.setNameFilterDisables(False)
        
        self.tree = QTreeView()
        self.tree.setModel(model)
        self.tree.setRootIndex(model.index(directory))
        self.tree.hideColumn(1)
        self.tree.hideColumn(2)
        self.tree.hideColumn(3)

    def createExplorerDock(self):
        self.explorer = QDockWidget("Файлы эксперимента", self)
        # self.explorer.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)

        sensorsWidget = QWidget(self)
        sensorsWidget.setMinimumWidth(self.width - (self.width / 1.1))

        self.setDirectoryToExplorer(QDir.currentPath())
        vBox = QVBoxLayout()
        vBox.addWidget(self.tree)

        sensorsWidget.setLayout(vBox)
        
        self.explorer.setWidget(sensorsWidget)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.explorer)


    def createToolBar(self):
        self.tools = QToolBar("Инструменты", self)
        self.tools.addSeparator()
        self.tools.setMovable(False)
        self.tools.setAllowedAreas(Qt.TopToolBarArea)
        self.tools.addAction("Добавить человека")
        self.tools.addAction("Заполнить параметры")
        self.addToolBar(Qt.TopToolBarArea, self.tools)


    def createLogDockWidget(self):
        self.log = QDockWidget("Лог", self)
        # self.log.setFeatures(QDockWidget.NoDockWidgetFeatures)
        # self.log.setAllowedAreas(Qt.NoDockWidgetArea)
        self.log.setAllowedAreas(Qt.BottomDockWidgetArea | Qt.TopDockWidgetArea)

        logWidget = QWidget(self)
        logWidget.setMinimumWidth(self.width / 1.1)
        
        self.log_text = QTextEdit(logWidget)
        # self.log_text.setEnabled(False)
        self.clear_log = QPushButton("Очистить лог", logWidget)
        logHBox = QHBoxLayout()
        logHBox.addStretch(1)
        logHBox.addWidget(self.clear_log)

        logVBox = QVBoxLayout()
        logVBox.addWidget(self.log_text)
        logVBox.addLayout(logHBox)
        logWidget.setLayout(logVBox)

        self.log.setWidget(logWidget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log)

        self.ip = '0'

        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.IP_Callback)
        self.log_timer.start(1000)
    
    def IP_Callback(self):
        host_name = socket.gethostname()
        host_ip = socket.gethostbyname(host_name)     
        if (self.ip != host_ip):
            self.ip = host_ip
            self.log_text.append('To connect IP : ' + host_ip )
            self.log_text.append('Start from port: 5555')

App = QApplication(sys.argv)
window = MainWindow()
sys.exit(App.exec())
