"""
Last Update:
- Se comprobo que se envian correctamente los numeros hacia el arduino

To Do:
- Chequear que los motores se muevan con esos numeros
- Chequear los numeros

"""
import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import pyqtSignal, QThread, QObject, QMutex, QWaitCondition
from PyQt5.QtTest import QTest
import socket
from threading import Thread
from bitstring import BitArray, BitStream, ConstBitStream
from time import time, ctime, strftime, localtime
import math
import coords
import logging
import serial
from serial.serialutil import SerialException
import numpy as np
import re
import serial.tools.list_ports


logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(message)s')
main1, main2 = uic.loadUiType("GUI/main.ui")


class MainWindow(main1, main2):

    get_coords_sig = pyqtSignal()
    ref_ready_sig = pyqtSignal()
    get_position_sig = pyqtSignal()
    move_sig = pyqtSignal(int,int)
    new_coords_sig = pyqtSignal(int,int)


    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.server = Server()
        try:
            self.controller = Controller()
            self.controller.start()
        except SerialException:
            logging.error("Puerto no conectado")
            self.server.socket_servidor.close()
            sys.exit()
        self.set_signals()
        self.refs = 0
        self.ra = 0
        self.dec = 0
        self.ref_coords_list = []
        self.ref_coords = {}

    def set_signals(self):
        self.server.coords_sig.connect(self.new_coords)
        self.ref_button.clicked.connect(self.set_reference)
        self.controller.send_coords_sig.connect(self.receive_coords)
        self.get_coords_sig.connect(self.controller.get_coords)
        self.ref_ready_sig.connect(self.activate_goto)
        self.goto_button.clicked.connect(self.goto_ask)
        self.controller.send_position_sig.connect(self.goto_move)
        self.get_position_sig.connect(self.controller.get_position)
        self.move_sig.connect(self.controller.move)
        self.new_coords_sig.connect(self.controller.new_coords)


    def new_coords(self, coords):
        ra, dec, time = coords
        self.RA_text.setText(ra)
        self.DEC_text.setText(dec)

    def closeEvent(self, event):
        self.server.socket_servidor.close()
        sys.exit()

    def set_reference(self):
        ra = self.RA_text.toPlainText()
        dec = self.DEC_text.toPlainText()
        if ra and dec:
            self.get_coords_sig.emit()
            self.RA_text.clear()
            self.DEC_text.clear()
            self.refs += 1
            self.ref_text.setText("{}/2".format(self.refs))
            self.ref_coords_list.append((ra, dec))

    def goto_ask(self):
        ra = self.RA_text.toPlainText()
        dec = self.DEC_text.toPlainText()
        if ra and dec:
            self.RA_text.clear()
            self.DEC_text.clear()
            self.get_position_sig.emit()
            ra, dec = self.transform.map_coords(ra, dec)
            self.obj_ra = ra
            self.obj_dec = dec


    def goto_move(self, ra, dec):
        ra = float(ra)
        dec = float(dec)
        dif_ra = int(self.obj_ra - ra)
        dif_dec = int(self.obj_dec - dec)
        logging.debug("Diferencia RA: {}".format(dif_ra))
        logging.debug("Diferencia DEC: {}".format(dif_dec))
        self.move_sig.emit(dif_ra,dif_dec)


    def activate_goto(self):
        self.ref_button.setEnabled(False)
        self.goto_button.setEnabled(True)
        self.transform = Transform(self.ref_coords)

    def receive_coords(self, ra, dec):
        tup = self.ref_coords_list.pop(0)
        self.ref_coords[tup] = (ra, dec)
        refs = len(self.ref_coords.items())
        if refs == 2:
            self.ref_ready_sig.emit()


class Controller(QThread):

    send_coords_sig = pyqtSignal(str, str)
    send_position_sig = pyqtSignal(str,str)

    


    def __init__(self):
        super().__init__()
        
        port = self.serial_port()
        baud = 9600
        self.serial = serial.Serial(port, baud, timeout=2)
        self.dec = 0
        self.ra = 0
        self.refs = []

    def serial_port(self):
        list = serial.tools.list_ports.comports()
        for element in list:
            if "usbmodem" in (element.device):
                puerto = element.device
                return puerto

    def run(self):
        while True:
            line = self.serial.readline().decode().rstrip().strip("\n")
            if line == "coords":
                ra = self.serial.readline().decode().rstrip().strip("\n")
                dec = self.serial.readline().decode().rstrip().strip("\n")
                self.send_coords_sig.emit(ra, dec)
            if line == "position":
                ra = self.serial.readline().decode().rstrip().strip("\n")
                dec = self.serial.readline().decode().rstrip().strip("\n")
                self.send_position_sig.emit(ra, dec)
            if line == "debug":
                data = self.serial.readline().decode().rstrip().strip("\n")
                logging.debug("Arduino: {}".format(data))
            if line == "move_ready":
                self.move_ready()


    def set_reference(self, ra, dec):
        ra = coords.hourStr_2_rad(ra)
        dec = coords.degStr_2_rad(dec)
        self.refs.append((ra, dec))

    def get_coords(self):
        self.serial.write("coords".encode())

    def get_position(self):
        self.serial.write("position".encode())

    def move(self, steps_ra, steps_dec):
        self.serial.write("move".encode())
        self.steps_ra = steps_ra
        self.steps_dec = steps_dec

    def move_ready(self):
        print("ready")
        self.serial.write("{} {}".format(self.steps_ra,self.steps_dec).encode())

    def new_coords(self,ra,dec):
        self.serial.write("new".encode())
        self.serial.write(ra)
        self.serial.write(dec)

class Server(QObject):

    coords_sig = pyqtSignal(tuple)

    def __init__(self):
        super().__init__()
        self.socket_servidor = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.socket_servidor.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind_and_listen()
        self.accept_connections()
        self.conectado = True

    def bind_and_listen(self):
        host = socket.gethostname()
        self.socket_servidor.bind(("localhost", 10001))
        self.socket_servidor.listen(5)
        logging.info("Servidor escuchando en {}:{}...".format(
            "localhost", 10001))

    def accept_connections(self):
        thread = Thread(target=self.accept_connections_thread)
        thread.start()

    def accept_connections_thread(self):
        logging.info("Servidor aceptando conexiones...")
        while True:
            client_socket, _ = self.socket_servidor.accept()
            self.socket_cliente = client_socket
            listening_client_thread = Thread(
                target=self.listen_client_thread, args=(client_socket,), daemon=True)
            listening_client_thread.start()

    def listen_client_thread(self, client_socket):
        logging.info("Servidor conectado a un nuevo cliente...")

        while self.conectado:
            data0 = client_socket.recv(160)
            if data0:
                data = ConstBitStream(bytes=data0, length=160)
                # print "All: %s" % data.bin

                msize = data.read('intle:16')
                mtype = data.read('intle:16')
                mtime = data.read('intle:64')

                # RA:
                ant_pos = data.bitpos
                ra = data.read('hex:32')
                data.bitpos = ant_pos
                ra_uint = data.read('uintle:32')

                # DEC:
                ant_pos = data.bitpos
                dec = data.read('hex:32')
                data.bitpos = ant_pos
                dec_int = data.read('intle:32')
                (sra, sdec, stime) = coords.eCoords2str(float("%f" %
                                                              ra_uint), float("%f" % dec_int), float("%f" % mtime))
                self.act_pos(coords.hourStr_2_rad(sra),
                             coords.degStr_2_rad(sdec))
                self.coords_sig.emit((sra, sdec, stime))

    def act_pos(self, ra, dec):
        (ra_p, dec_p) = coords.rad_2_stellarium_protocol(float(ra), float(dec))

        # Number of times that Stellarium expects to receive new coords
        # //Absolutly empiric..
        times = 10
        for i in range(times):
            self.move(ra_p, dec_p)

    # Sends to Stellarium equatorial coordinates
    #
    #  Receives the coordinates in float format. Obtains the timestamp from local time
    #
    # \param ra Ascensión recta.
    # \param dec Declinación.
    def move(self, ra, dec):
        msize = '0x1800'
        mtype = '0x0000'
        aux_format_str = 'int:64=%r' % time()
        localtime = ConstBitStream(aux_format_str.replace('.', ''))

        sdata = ConstBitStream(msize) + ConstBitStream(mtype)
        sdata += ConstBitStream(intle=localtime.intle,
                                length=64) + ConstBitStream(uintle=ra, length=32)
        sdata += ConstBitStream(intle=dec, length=32) + \
            ConstBitStream(intle=0, length=32)

        self.buffer = sdata
        self.socket_cliente.send(self.buffer.bytes)


class Transform:

    def __init__(self, dic):
        self.dic = dic
        self.create_matrix()

    def create_matrix(self):
        star1_stell, star1_mount = self.dic.popitem()
        star2_stell, star2_mount = self.dic.popitem()


        stell_1 = self.get_vector(star1_stell)
        mount_1 = self.get_vector(star1_mount)
        stell_2 = self.get_vector(star2_stell)
        mount_2 = self.get_vector(star2_mount)
        stell_3 = np.cross(stell_1, stell_2)
        mount_3 = np.cross(mount_1, mount_2)
        matrix_stell = np.concatenate(
            (stell_1.T, stell_2.T, stell_3.T), axis=1)
        matrix_mount = np.concatenate(
            (mount_1.T, mount_2.T, mount_3.T), axis=1)
        matrix_stell_inv = np.linalg.inv(matrix_stell)
        matrix = np.dot(matrix_mount, matrix_stell_inv)
        self.matrix = matrix

    def get_vector(self, tup):
        ra, dec = tup
        try:
            ra = float(ra)
            dec = float(dec)
        except ValueError:
            ra = coords.hourStr_2_rad(ra)
            dec = coords.degStr_2_deg(dec)
            dec = math.radians(dec)

        else:
            ra = math.radians(ra)
            dec = math.radians(dec)

        vec1 = math.cos(dec) * math.cos(ra)
        vec2 = math.cos(dec) * math.sin(ra)
        vec3 = math.sin(dec)
        vector = np.array([[vec1, vec2, vec3]])
        return vector

    def map_coords(self, ra_str, dec_str):
        tup = (ra_str, dec_str)
        vector = self.get_vector(tup)
        mapped = np.dot(self.matrix, vector.T)
        comp1 = mapped[0][0]
        comp2 = mapped[1][0]
        comp3 = mapped[2][0]
        ra = math.degrees(math.atan2(comp2, comp1))
        dec = math.degrees(math.asin(comp3))
        if ra < 0:
            ra = ra + 360
        if dec < 0:
            dec = dec + 360
        return (ra, dec)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    app_gui = MainWindow()
    app_gui.show()
    sys.exit(app.exec_())
