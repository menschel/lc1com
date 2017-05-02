#lc1com.py
# (C) 2017 Patrick Menschel

import serial
import threading
import queue
import logging
import time
import sys
import pickle

LC1_STREAM_DATA_FRAME_START = 0xB2
LC1_REQUEST_DATA_FRAME_START = 0xA2

LC1_CMD_REQUEST_ID = 0xF3
LC1_CMD_REQUEST_VERSION = 0xCE

LC1_CMD_REQUEST_PROGRAMMER_INIT = 0x83

LC1_DEVICE_ID = 0x82
LC1_CHANNEL_ID_O2 = 0x47
LC1_CHANNEL_ID_HEATER = 0x53
LC1_CHANNEL_ID_ERROR = 0x5B


LC1_FRAME_START_CHARACTERS = [LC1_STREAM_DATA_FRAME_START,LC1_REQUEST_DATA_FRAME_START]

LC1_MODE_HEATING = "heating"
LC1_MODE_O2MEASURE = "o2measure"
LC1_MODE_LAMBDAMEASURE = "lambdameasure"



def floatvaluetobytes(fval):
    val = int(fval*10)
    lsb = val & 0x7f
    msb = (val & 0x3F80) >> 7
    return bytes((msb,lsb))

def bytestofloatvalue(bs):
    ba = bytearray(bs)
    val =(ba[0]<<7)+ba[1]
    fval = float(val)/10
    return fval

def gettimestamp(): 
    return time.time()



def generate_message(val1,
                     val2,
                     msg_type=LC1_STREAM_DATA_FRAME_START,
                     device_id=LC1_DEVICE_ID,
                     ):
    msg = bytearray()
    msg.append(msg_type)
    msg.append(device_id)
    msg.extend(floatvaluetobytes(val1))
    msg.extend(floatvaluetobytes(val2))
    return msg





def generate_message_o2_lambda(lambda_val,o2_val,device_id=LC1_DEVICE_ID):
    return generate_message(val1=(lambda_val - 0.5)*100,
                            device_id=device_id,
                            )



def generate_message_heater(heater_val,device_id=LC1_DEVICE_ID):
    return generate_message(val1=1064.3,
                            val2=heater_val*100,
                            device_id=device_id,
                            )


def interpret_message(msg):
    if len(msg) >= 2:
        msg_type = msg[0]
        device_id = msg[1]
        if msg_type == LC1_STREAM_DATA_FRAME_START:
            val1 = bytestofloatvalue(msg[2:4])
            val2 = bytestofloatvalue(msg[4:6])
            return {"msg_type":msg_type,
                    "device_id":device_id,
                    "val1":val1,
                    "val2":val2,
                    "raw":msg,
                    }

        elif msg_type == LC1_REQUEST_DATA_FRAME_START:
            data = msg[2:]
            return {"msg_type":msg_type,
                    "device_id":device_id,
                    "data":data,
                    "raw":msg,
                    }
    return None
        
    

  
class lc1com():
    def __init__(self,port):
        if isinstance(port,serial.Serial):
            self.ser = port
        elif isinstance(port,str):
            self.ser = serial.Serial(port=port,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.1)
        self.mutex = threading.Lock()
        self.name = "Device"
        self.status = "init"
        self.o2value = None
        self.lambdavalue = None
        self.heatervalue = None
        self._init_logger()
        self.rxbuff = bytearray()
        self.requests = queue.Queue()
        
        self.rxhandler = threading.Thread(target=self.handlerx)
        self.rxhandler.setDaemon(True)
        self.rxhandler.start()
        self._log_info('Init Complete')
        self.connect()
        
    def _init_logger(self):
        self._logger = logging.getLogger('lc1com')
        self._logger.setLevel(logging.DEBUG)
        self._fh = logging.FileHandler('lc1com.log')
        self._fh.setLevel(logging.INFO)
        self._ch = logging.StreamHandler()
        self._ch.setLevel(logging.ERROR)
        self._formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self._fh.setFormatter(self._formatter)
        self._ch.setFormatter(self._formatter)
        self._logger.addHandler(self._fh)
        self._logger.addHandler(self._ch)
        self._log_info('Logger has been initialized')
        return
    
    def _log_info(self,msg):
        if self._logger:
            self._logger.info(msg)
        return
                
    def _log_error(self,msg):
        if self._logger:
            self._logger.error(msg)
        return
    
    def _log_debug(self,msg):
        if self._logger:
            self._logger.debug(msg)
        return

    def get_status(self):
        return self.status

    def get_o2value(self):
        return self.o2value
    
    def get_lambdavalue(self):
        return self.lambdavalue

    def get_heater(self):
        return self.heatervalue

    def get_name(self):
        return self.name

    def __str__(self):
        n = self.get_name()
        stat = self.get_status()
        s = ""
        if stat == LC1_MODE_HEATING:
            val = self.get_heater()
            s = "{0} {1} {2:3.1%}       ".format(n,stat,val)
        elif stat == LC1_MODE_O2MEASURE:
            val = self.get_o2value()
            s = "{0} {1} {2:3.1%}       ".format(n,stat,val)
        elif stat == LC1_MODE_LAMBDAMEASURE:
            val = self.get_lambdavalue()
            s = "{0} {1} {2:3.3}       ".format(n,stat,val)        
        return s                    
    
    def request(self,service):
        msg = bytearray()
        msg.append(service)
        self.ser.write(msg)
        return self.requests.get()

    def requestid(self):
        return self.request(LC1_CMD_REQUEST_ID)
                
    def requestversion(self):
        return self.request(LC1_CMD_REQUEST_VERSION)    
    
    def handlerx(self):
        while True:
            indata = self.ser.read(self.ser.inWaiting())
            if indata:
                self.onrxdata(indata)
            else:
                time.sleep(0.1)
        return

    def onrxdata(self,data):
        self._log_debug('Serial Read {0}'.format(' '.join(['{0:02x}'.format(x) for x in data])))
        self.rxbuff.extend(data)
        lastidx = 0
        for idx,b in enumerate(self.rxbuff):
            if b in LC1_FRAME_START_CHARACTERS:
                rx_msg = self.rxbuff[lastidx:idx]
                if rx_msg:
                    self.onrxmsg(msg=rx_msg)
                lastidx = idx
        self.rxbuff = self.rxbuff[lastidx:] 
                     
        return

    def onrxmsg(self,msg):
        self._log_debug('RX Message {0}'.format(' '.join(['{0:02x}'.format(x) for x in msg])))
        msg_dct = interpret_message(msg)
        if msg_dct:
            tp = msg_dct["msg_type"]            
            if tp == LC1_STREAM_DATA_FRAME_START:
                self.onmeasurement(msg_dct)
            
            elif tp == LC1_REQUEST_DATA_FRAME_START:
                self.requests.put(msg_dct)
                
            else:
                raise NotImplementedError("Frame Type {0} not handled".format(tp))

    def onmeasurement(self,msg_dct):
        val1 = msg_dct["val1"]
        val2 = msg_dct["val2"]
            with self.mutex:
                self.status = LC1_MODE_HEATING
                self.lambdavalue = None
                self.o2value = None
                self.heatervalue = val2/100 
            with self.mutex:
                self.status = LC1_MODE_O2MEASURE
                self.lambdavalue = (val1/100)+0.5
                self.o2value = val2 /100
        else:
            with self.mutex:
                self.status = LC1_MODE_LAMBDAMEASURE
                self.lambdavalue = (val1/100)+0.5
                self.o2value = val2 /100
        return


    def connect(self):
        dev_id = self.requestid()["data"]
        print("Connected to {0}".format(dev_id))
        dev_version = self.requestversion()["data"]
        print("Version {0}".format(dev_version))
        return




class lc1simulator():
    
    def __init__(self,port):
        self._init_logger()
        if isinstance(port,serial.Serial):
            self.ser = port
        elif isinstance(port,str):
            self.ser = serial.Serial(port=port,baudrate=19200,bytesize=serial.EIGHTBITS,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.1)
        self.ID =      bytes.fromhex('a2850173110a4c4331200532') 
        self.VERSION = bytes.fromhex('a285014e4c432d315f310000')
        self.currenttxmessage = bytes.fromhex('b2 82 53 13 01 73') 
        self.manualcommandqueue = queue.Queue(maxsize=1)
        self.statusqueue = queue.Queue()
        self.simmode = 'auto'
        self.heater = 0.0
        self.lambda_value = 1.0
        self.o2 = 0.2094
        self.cycletime = 0.1
        self.statehandler = threading.Thread(target=self.simulate)
        self.txqueue = queue.Queue()
        self.txhandler = threading.Thread(target=self.handletx)
        self.txhandler.setDaemon(True)
        self.rxhandler = threading.Thread(target=self.handlerx)
        self.rxhandler.setDaemon(True)
        self.rxhandler.start()
        self.txhandler.start()
        self.statehandler.start()
        self._log_info('Init Complete')
    
    def __del__(self):
        self.statehandler.stop()
        self.ser.close()
        self._log_info('Destructor Complete')
        return
        
    def _init_logger(self):
        self._logger = logging.getLogger('lc1simulator')
        self._logger.setLevel(logging.DEBUG)
        self._fh = logging.FileHandler('lc1simulator.log')
        self._fh.setLevel(logging.INFO)
        self._ch = logging.StreamHandler()
        self._ch.setLevel(logging.ERROR)
        self._formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        self._fh.setFormatter(self._formatter)
        self._ch.setFormatter(self._formatter)
        self._logger.addHandler(self._fh)
        self._logger.addHandler(self._ch)
        self._log_info('Logger has been initialized')
        return
    
    def _log_info(self,msg):
        if self._logger:
            self._logger.info(msg)
        return
                
    def _log_error(self,msg):
        if self._logger:
            self._logger.error(msg)
        return
    
    def _log_debug(self,msg):
        if self._logger:
            self._logger.debug(msg)
        return    
        
    def handlerx(self):
        self._log_debug('handlerx started')
        while True:
            cmd = self.ser.read(self.ser.inWaiting())
            self._log_debug('Serial Read {0}'.format(' '.join(['{0:02x}'.format(x) for x in cmd])))
            if cmd:
                self.on_cmd(cmd)
            else:
                time.sleep(0.1)
        return             
            
    def on_cmd(self,cmd):
        cmds = bytearray(cmd)
        for c in cmds:
            if c == LC1_CMD_REQUEST_ID:
                self._log_info('ID Request')
                self.write_id()
                   
            elif c == LC1_CMD_REQUEST_VERSION:
                self._log_info('VERSION Request')
                self.write_version()
            else:
                self._log_error('Unknown Command {0}'.format(cmd))
        return
    
    def write_id(self):
        self.transmit(self.ID)
        self._log_info('Wrote ID {0}'.format(self.ID))
        return 
    
    def write_version(self):
        self.transmit(self.VERSION)
        self._log_info('Wrote VERSION {0}'.format(self.VERSION))
        return
    
    def handletx(self):
        self._log_debug('handletx started')
        while True:
            nexttxmessage = self.txqueue.get()
            self.ser.write(nexttxmessage)
            self._log_debug('Serial Write {0}'.format(' '.join(['{0:02x}'.format(x) for x in nexttxmessage])))
        return
    
    def simulate(self):
        self.simmode = 'auto'
        nextmsg = generate_message_heater(heater_val=self.heater)
        while True:
            if not self.manualcommandqueue.empty:
                self.simmode = 'manual'
                self.simmode,self.lc1mode,val=self.manualcommandqueue.get()
                if self.lc1mode == 'heating':
                    self.heater = val
                elif self.lc1mode == 'o2measure':
                    self.o2 = val
                                             
            if self.simmode == 'auto':
                if self.simmode == 'init':                
                    self.heater = 0
                    self.o2 = 0
                    self._log_info('Simulate Init Complete - Start Heating Next')
                    self.lc1mode = 'heating'
                    nextmsg = generate_message_heater(heater_val=self.heater)
                     
                elif self.lc1mode == 'heating':
                    if self.heater >= 1:
                        self._log_info('Simulate Heating Complete')
                        self.lc1mode = 'o2measure'
                         
                    else:
                        self.heater += 0.05               
                        nextmsg = generate_message_heater(heater_val=self.heater)
                         
                elif self.lc1mode == 'o2measure':
                        self.lc1mode = 'lambdameasure'
                        self.lambda_value = 1.5
                        nextmsg = generate_message_o2_lambda(lambda_val=self.lambda_value, o2_val=self.o2)
                    else:
                        self.o2 -= 0.001
                        self.lambda_value = 9.618
                        
                        nextmsg = generate_message_o2_lambda(lambda_val=self.lambda_value, o2_val=self.o2)
                    
                elif self.lc1mode == 'lambdameasure':
                    self.lambda_value -= 0.01
                    if self.lambda_value < 0.5:
                        self.lc1mode = 'o2measure'
                        self.o2=0.2094
                    nextmsg = generate_message_o2_lambda(lambda_val=self.lambda_value, o2_val=self.o2)
                                 
            elif self.simmode == 'manual':
                 
                if self.lc1mode == 'heating':
                    nextmsg = generate_message_heater(heater_val=self.heater)
                     
                elif self.lc1mode == 'o2measure':
                    nextmsg = generate_message_o2_lambda(lambda_val=self.lambda_value, o2_val=self.o2)
            
            self.transmit(msg=nextmsg)
            time.sleep(0.1)

    def transmit(self,msg):
        self.currenttxmessage = msg
        self.txqueue.put(self.currenttxmessage)
        return

    
    def set_manual_heater(self,val):
        self.manualcommandqueue.put(('manual','heating',val))
        return
    
    def set_manual_o2(self,val):
        self.manualcommandqueue.put(('manual','o2measure',val))
        return    
    
    def set_auto(self):
        self.manualcommandqueue.put(('auto','init',0))
        return
    
    def get_status(self):
        return {'simmode':self.simmode,
                'lc1mode':self.lc1mode,
                'heater':self.heater,
                'lambda':self.lambda_value,
                'o2':self.o2}
    



def selftest(testmode = 'simulatelc1',port="/dev/ttyS0"):
    try:
            
        if testmode == 'readlc1':
            mylc1com = lc1com(port=port)
            while True:
                time.sleep(0.1)

        elif testmode == 'lc1simulator':
            lc1sim = lc1simulator(port=port) 
            while True:
                print('Sim {simmode} LC1 {lc1mode} Heater {heater:3.1%} O2 {o2:2.1%} Lambda {lambda:1.1f}'.format_map(lc1sim.get_status()))
                time.sleep(1)
                                    
                   
    except KeyboardInterrupt:
        sys.exit(0)
        

if __name__ == "__main__":
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-c", "--command", dest="command",
                      default="readlc1",
                      help="COMMAND to execute", metavar="COMMAND")
    parser.add_option("-p", "--port", dest="port",
                      default="/dev/ttyS0",
                      help="PORT device to use", metavar="PORT")
    (options, args) = parser.parse_args()
   
    selftest(testmode=options.command,port=options.port)
     

