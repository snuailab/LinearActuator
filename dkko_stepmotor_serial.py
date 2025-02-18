import serial
import time


class DKKO_STEPMOTOR_DRIVER:
    ser = None;
    def __init__(self,port_name ='/dev/ttyACM0'):
        ser = serial.Serial(port = port_name, 
                            baudrate=115200, 
                            parity='N',
                            stopbits=1,
                            bytesize=8,
                            timeout=8,
                            )
        self.ser = ser;
        pass
    
    def connect(self):
        if(not self.ser.isOpen()):
            return "";
        self.ser.timeout = 1;
        res=self.ser.readline();
        self.ser.timeout = 8;
        try:
            return res.decode();
        except:
            return ""
        

    def close(self):
        if(self.ser.isOpen()):
            self.ser.close();
    
    def init(self):
        motor_state = None;
        motor_pos   = None;
        data=bytes(bytearray([0x0F,ord('i'),0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(9+1);
        if(receive_data.__len__()>0):
            if(receive_data[1]==ord('i')):
                return True;
        return False
    
    def get_pos(self):
        motor_state = None;
        motor_pos   = None;
        data=bytes(bytearray([0x0F,ord('g'),0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(9+1);
        if(receive_data.__len__()>0):
            if(receive_data[1]==ord('g')):
                motor_state = int.from_bytes(receive_data[3:5], byteorder='little',signed=True);
                motor_pos   = int.from_bytes(receive_data[5:7], byteorder='little',signed=True);
        return motor_state,motor_pos;
    
    def set_profile(self,pos_x:int,vel:int,acc:int):
        pos_arr     = bytearray(pos_x.to_bytes(2,byteorder='little',signed=True));
        vel_arr     = bytearray(vel.to_bytes(2, byteorder='little',signed=True));
        acc_arr     = bytearray(acc.to_bytes(2, byteorder='little',signed=True));

        count = 0
        data=bytes(bytearray([0x0F,ord('p'),0x06,pos_arr[0],pos_arr[1],0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(6+1);
        if(receive_data.__len__()>0):
            if((receive_data[1]==ord('p')) and (receive_data[3] == 1)):
                count +=1;
        #time.sleep(0.1);
        data=bytes(bytearray([0x0F,ord('v'),0x06,vel_arr[0],vel_arr[1],0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(6+1);
        if(receive_data.__len__()>0):
            if((receive_data[1]==ord('v')) and (receive_data[3] == 1)):
                count +=1;
        #time.sleep(0.1);
        data=bytes(bytearray([0x0F,ord('a'),0x06,acc_arr[0],acc_arr[1],0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(6+1);
        if(receive_data.__len__()>0):
            if((receive_data[1]==ord('a')) and (receive_data[3] == 1)):
                count +=1;
        #time.sleep(0.1);
        if(count ==3):
            return True;
        return False;

    def get_profile(self):

        p_state    = None;
        p_to_pos   = None;
        p_to_vel   = None;
        p_to_acc   = None;

        data=bytes(bytearray([0x0F,ord('q'),0x0f]));
        self.ser.write(data);
        self.ser.write
        receive_data = self.ser.read(13+1);
        if(receive_data.__len__()>0):
            if(receive_data[1]==ord('q')):
                p_state     = int.from_bytes(receive_data[3:5], byteorder='little',signed=True);
                p_to_pos    = int.from_bytes(receive_data[5:7], byteorder='little',signed=True);
                p_to_vel    = int.from_bytes(receive_data[7:9], byteorder='little',signed=True);
                p_to_acc    = int.from_bytes(receive_data[9:11], byteorder='little',signed=True);
        return p_state,p_to_pos,p_to_vel,p_to_acc;
    

    def set_move(self):
        data=bytes(bytearray([0x0F,ord('m'),0x0f]));
        self.ser.write(data);
        receive_data = self.ser.read(6+1);
        if(receive_data.__len__()>0):
            if((receive_data[1]==ord('m')) and (receive_data[2] == 1)):
                return True;
                
                
        return False;

    
        


if __name__ == "__main__":
    motor=DKKO_STEPMOTOR_DRIVER("/dev/cu.usbmodem1101");
    res=motor.connect();
    print(res);
    ones = 0;
    time.sleep(2);
    motor.init();
    s = time.time();
    postion_arr = [100,300,400,500,600,700];
    postion_idx = 0;
    while(True):
        f = time.time();
        if((f-s)>60.0):
            break;

        state,pos=motor.get_pos();
        print("motor state : ",state, "motor pos : ",pos);
        
        if((ones==0) and (state==5)):
            ones = 1;
            res = motor.set_profile(postion_arr[postion_idx%postion_arr.__len__()]+60,300,600);
            print("set_profile : ",res);
        elif((ones==1) and (state==5)):
            ones = 2;
            res = motor.set_move();
            print("set_move : ",res);
        elif((ones==2) and (state==5)):
            p_state,p_to_pos,p_to_vel,p_to_acc = motor.get_profile();
            if(p_state==0):
                postion_idx +=1;
                ones = 0;
            #print("p_state : ",p_state, ", p_to_pos : ",p_to_pos,", p_to_vel : ",p_to_vel, ", p_to_acc", p_to_acc);
    
            

    motor.close();