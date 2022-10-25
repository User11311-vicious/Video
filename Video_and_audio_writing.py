#!/usr/bin/python3
from curses import COLOR_BLACK
from cv2 import CAP_V4L2, rectangle, putText, VideoCapture, VideoWriter, VideoWriter_fourcc, CAP_PROP_FOURCC, CAP_V4L, resize, FONT_HERSHEY_SIMPLEX, LINE_AA, destroyAllWindows, imshow, waitKey, CAP_PROP_FPS
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from bluetooth import *
from datetime import datetime
import csv
import serial
from multiprocessing import Process, shared_memory
from time import *
import socket
from RPi import GPIO

#settings_of_memory___and___gpio
pedal = shared_memory.ShareableList([0, 0, 0, 0, 0, 0], name="pedal") #pedals
indi = shared_memory.ShareableList([0, ' '*128, ' '*128, ' '*128, ' '*128, ' '*128, ' '*128], name="indik")
end = shared_memory.ShareableList([0, 0, 0, 0, True], name="en")
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT) #memory_zoomer
GPIO.setup(16, GPIO.OUT) #memory_diod
GPIO.setup(19, GPIO.OUT) #esp_zoomer
GPIO.setup(20, GPIO.OUT) #esp_diod
GPIO.setup(17, GPIO.OUT) #video_zoomer
GPIO.setup(18, GPIO.OUT) #video_diod
GPIO.setup(5, GPIO.OUT) #power_diod
GPIO.setup(6, GPIO.OUT) #mpu_and_gps_diod
##################################

def dates():
    #excel_nastroyki
    fieldnames = ['Date', 'Speed, km/h', 'X-axis acceleration, m/s^2', 'Y-axis acceleration, m/s^2', 'Angular acceleration, rad/s^2', 'Gas', 'Brake', 'GPS']
    #nastroyki_bluetooth
    addr = "24:62:AB:E1:96:7A"
    channel = 1
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.connect((addr, channel))
    ##########################
    #mpu_settings
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS, 
        address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
        address_mpu_slave=None, 
        bus=1,
        gfs=GFS_1000, 
        afs=AFS_8G, 
        mfs=AK8963_BIT_16, 
        mode=AK8963_MODE_C100HZ)
        
    mpu.calibrateMPU6500()
    mpu.configure()
    # ###################

    #nastroyka gps
    ser = serial.Serial('/dev/ttyS0')
    ser.baudrate = 9600
    gas, tormos, velocity, info_gps, v_0, t = '', '', 0, '', 0, 0.9
    ##############################################
    if end[1] == 0:
        GPIO.output(6, True)
        sleep(1)
        GPIO.output(6, True)
        sleep(1)
        GPIO.output(6, True)
        sleep(1)
        end[1] = 1   
    while 1:
        try:
            if end[4] == True:
                csv_main = '/media/pi/Новый том/' + datetime.today().strftime('%Y.%m.%d_%H.%M.%S') + '.csv'
                end[4] = 2
            if indi[0] == 1:
                addr = "24:62:AB:E1:96:7A"
                channel = 1
                s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                s.connect((addr, channel))
                indi[0] = 0
            ##################
            Accelerometer = mpu.readAccelerometerMaster()
            a_x = round(Accelerometer[0], 1)
            a_y = round(Accelerometer[1], 1)
            #########################################
            Gyroscope = mpu.readGyroscopeMaster()
            g_x = round(Gyroscope[0], 1)
            ######################################
            velocity = v_0 + a_x*t
            velocity = round(velocity, 2)
            if velocity < 0:
                velocity *= -1
            v_0 = velocity
            pedal[2] = a_x
            pedal[3] = a_y
            pedal[4] = g_x
            pedal[5] = velocity
            #########
            data = s.recv(1024)
            data = data.decode('utf-8')
            data = data.split('|')
            if data == '' or data == None:
                end[3] = 1
            GPIO.output(20, True)
            #tormos_starts...
            if 'Brake depressed' in data:
                tormos = 'Brake depressed'
                pedal[0] = 1
            elif 'Brake pressed' in data:
                tormos = 'Brake pressed'
                pedal[0] = 2
            elif 'Brake ready' in data:
                tormos = 'Brake ready'
                pedal[0] = 3
            #gas_starts...
            if 'Gas depressed' in data:
                gas = 'Gas depressed'
                pedal[1] = 1
            elif 'Gas pressed' in data:
                gas = 'Gas pressed'
                pedal[1] = 2
            elif 'Gas ready' in data:
                gas = 'Gas ready'
                pedal[1] = 3
            data_gps = ser.readline()
            info_gps = info_gps + data_gps.decode('ISO-8859-1')
            if info_gps == '' or info_gps == None:
                end[2] = 1
            with open(csv_main, mode="a+", encoding='ISO-8859-1') as a:
                writer = csv.DictWriter(a, fieldnames=fieldnames)
                if end[4] == 2:
                    writer.writeheader()
                    end[4] = False 
                writer.writerows([{'Date': datetime.now(), 'Speed, km/h': str(velocity), 'X-axis acceleration, m/s^2': a_x, 'Y-axis acceleration, m/s^2': a_y, 'Angular acceleration, rad/s^2': str(g_x), 'Gas': gas, 'Brake': tormos}])
                if 'GNGLL' in data_gps.decode('ISO-8859-1'):
                    writer.writerows([{'Date': datetime.now(), 'Speed, km/h': str(velocity), 'X-axis acceleration, m/s^2': a_x, 'Y-axis acceleration, m/s^2': a_y, 'Angular acceleration, rad/s^2': str(g_x), 'Gas': gas, 'Brake': tormos, 'GPS': info_gps}])        
                    info_gps = ''
        except OSError:
            indi[0] = 1
            if end[3] == 1:
                print('перевоткни есп')
                GPIO.output(19, True)
                GPIO.output(20, True)
                GPIO.output(19, False)
                GPIO.output(20, False)
                end[3] = 0
                sleep(0.25)
        except UnicodeDecodeError:
            if end[2] == 1:
                GPIO.output(6, True)
                GPIO.output(6, False)
                sleep(0.25)
                end[2] = 0
    
def save_video(number, vid):
    if end[1] == 1:
        name_vid = 'Video' + vid
        ms = datetime.today().strftime('%f') + '0000'
        ms = ms[:3]
        name = datetime.today().strftime('%Y.%m.%d_%H.%M.%S')
        tiime = name + ms + vid
        if vid == '_v1':
            end[4] = True
        title_video = "/home/pi/Новый том/" + tiime + '_' + '.avi'
        title__video = tiime + '_.avi'
        title__video_1 = tiime + '.avi'
        cap = VideoCapture(number, CAP_V4L2)
        fourcc = VideoWriter_fourcc('M','J','P','G')
        cap.set(CAP_PROP_FOURCC,VideoWriter_fourcc('M','J','P','G'))
        out = VideoWriter(title_video, fourcc, 20.0, (1080, 720))
        i = 0
        print('start')
        while 1:
            try:
                _, frame = cap.read()
                re_frame = resize(frame, (1080,720))
                rectangle(re_frame, pt1=(900,0), pt2=(1040,20), color=(255, 255, 255), thickness= -1)
                putText(re_frame, name_vid, (900,20), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2, LINE_AA)
                if indi[0] == 1:
                    rectangle(re_frame, pt1=(0,50), pt2=(250,100), color=(255, 255, 255), thickness= -1)
                    putText(re_frame, 'Sensor error', (0,80), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                else:
                    if pedal[0] == 1:
                        rectangle(re_frame, pt1=(0,50), pt2=(270,100), color=(255, 255, 255), thickness= -1)
                        putText(re_frame, 'Brake depressed', (0,80), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                    elif pedal[0] == 2:
                        rectangle(re_frame, pt1=(0,50), pt2=(240,100), color=(0, 0, 255), thickness= -1)
                        putText(re_frame, 'Brake pressed', (0,80), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                    elif pedal[0] == 3:
                        rectangle(re_frame, pt1=(0,50), pt2=(200,100), color=(0, 255, 255), thickness= -1)
                        putText(re_frame, 'Brake ready', (0,80), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                    if pedal[1] == 1:
                        rectangle(re_frame, pt1=(0,0), pt2=(240,50), color=(255, 255, 255), thickness= -1)
                        putText(re_frame, 'Gas depressed', (0,30), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                    elif pedal[1] == 2:
                        rectangle(re_frame, pt1=(0,0), pt2=(200,50), color=(0, 0, 255), thickness= -1)
                        putText(re_frame, 'Gas pressed', (0,30), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                    elif pedal[1] == 3:
                        rectangle(re_frame, pt1=(0,0), pt2=(170,50), color=(0, 255, 255), thickness= -1)
                        putText(re_frame, 'Gas ready', (0,30), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2)
                rectangle(re_frame, pt1=(0,120), pt2=(270,150), color=(255, 255, 255), thickness= -1)
                rectangle(re_frame, pt1=(0,170), pt2=(250,203), color=(255, 255, 255), thickness= -1)
                rectangle(re_frame, pt1=(0,220), pt2=(250,250), color=(255, 255, 255), thickness= -1)
                rectangle(re_frame, pt1=(0,270), pt2=(190,300), color=(255, 255, 255), thickness= -1)
                putText(re_frame, 'X: ' + str(pedal[2]) + ' m/s^2', (0,145), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2, LINE_AA)
                putText(re_frame, 'Y: ' + str(pedal[3]) + ' m/s^2', (0,197), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2, LINE_AA)
                putText(re_frame, str(pedal[4]) + ' rad/s^2', (0,245), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2, LINE_AA)
                putText(re_frame, str(pedal[5]) + ' km/h', (0,305), FONT_HERSHEY_SIMPLEX, 1, COLOR_BLACK, 2, LINE_AA)
                out.write(re_frame)
                imshow('frame', re_frame)
                if waitKey(1) == ord(' ') or i > 1980:
                    print('end')
                    if vid == '_v1':
                        indi[1] = title__video
                        indi[2] = title__video_1
                    elif vid == '_v2':
                        indi[3] = title__video
                        indi[4] = title__video_1
                    if vid == '_v3':
                        indi[5] = title__video
                        indi[6] = title__video_1
                    end[0] = 1
                    break
                i += 1
            except:
                GPIO.output(18, True)
                GPIO.output(17, False)
                sleep(0.25)
        cap.release()
        out.release()
        destroyAllWindows()

def video_with_audio(ar):
    try:
        if end[0] == 1:
            if ar == 1:
                video_1 = '/media/pi/Новый\ том/' + str(indi[1])
                title_video_1 = '/media/pi/Новый\ том/' + str(indi[2])
                os.system('ffmpeg -i ' + video_1 + ' -vf setpts=PTS*2.28 ' + title_video_1)
                sleep(1)
                os.remove('/media/pi/Новый том/' + str(indi[1]))
            elif ar == 2:
                video_2 = '/media/pi/Новый\ том/' + str(indi[3])
                title_video_2 = '/media/pi/Новый\ том/' + str(indi[4])
                os.system('ffmpeg -i ' + video_2 + ' -vf setpts=PTS*2.28 ' + title_video_2)
                sleep(1)
                os.remove('/media/pi/Новый том/' + str(indi[3]))
            elif ar == 3:
                video_3 = '/media/pi/Новый\ том/' + str(indi[5])
                title_video_3 = '/media/pi/Новый\ том/' + str(indi[6])
                os.system('ffmpeg -i ' + video_3 + ' -vf setpts=PTS*2.28 ' + title_video_3)
                sleep(1)
                os.remove('/media/pi/Новый том/' + str(indi[5]))
                end[0] = 0
    except BaseException as err:
        print(err)

for i in range(3):
    GPIO.output(5, True)
    GPIO.output(5, False)
    sleep(1)
GPIO.output(5, True)
head = True
sleep(3)
for i in range(30000):
    while 1:
        st = os.statvfs('/media/pi/Новый том/')
        du = st.f_bsize * st.f_bavail / 1024 / 1024 / 1024
        if du > 0.01:
            GPIO.output(16, True)
            proc = Process(target=video_with_audio, args=(1,), daemon=True)
            proc1 = Process(target=video_with_audio, args=(2,), daemon=True)
            proc2 = Process(target=video_with_audio, args=(3,), daemon=True)
            p1 = Process(target=dates)
            p3 = Process(target=save_video, args=(0, '_v1'))
            p5 = Process(target=save_video, args=(8, '_v2'))
            p7 = Process(target=save_video, args=(4, '_v3'))
            if head == True:
                p1.start()
            if head == False:
                proc.start()
                proc1.start()
                proc2.start()
            p3.start()
            p5.start()
            p7.start()
            p3.join()
            p5.join()
            p7.join()
            head = False
            if head == False:
                break
        else:
            while True:
                GPIO.output(19, True)
                GPIO.output(20, True)
                GPIO.output(19, False)
                sleep(0.25)

pedal.shm.close()
pedal.shm.unlink()
indi.shm.close()
indi.shm.unlink()
end.shm.close()
end.shm.unlink()
