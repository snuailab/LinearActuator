from tkinter import *
from tkinter import filedialog
from tkinter import ttk
from tkinter import messagebox


import yaml
from dkko_stepmotor_serial import *

class gap_inspection_main_gui:
    cfg = None;
    gui_list = {}
    motor = None;
    windows_serial_port_list = ['COM'+str(i) for i in range(1,16)];
    windows_serial_port_idx = 0;
    count = 1;
    tp_list = [];
    tp_count = 0;
    tp_live = True;
    tp_mode = 0;

    def __init__(self) -> None:
        default_x = 25;
        default_y = 40;
        
        self.tk = Tk(className=' SNUAILAB-MOTOR ');
        self.tk.geometry("1280x450")
        self.tk.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        #title
        label = Label(self.tk,text='SNUAILAB - MotorDrive')
        label.pack()

        #config val
        self.create_label(None         , default_x+1050     , default_y - 40 , "Config");
        self.create_button("bt_SAVE"   , default_x+1100     , default_y - 5  , "SAVE" ,command=self.save_event);
        self.create_button("bt_LOAD"   , default_x+1000     , default_y - 5  , "LOAD" ,command=self.load_event);
        
        
  
        #serial communication val
        self.create_label(None              , default_x         , default_y-40   , "Device Name");
        self.create_entry("ent0"            , default_x         , default_y      , "/dev/cu.usbmodem1101",width=20);
        self.create_button("bt0"            , default_x+170     , default_y - 5  , "connect" ,command=self.connect);
        self.create_label("serial_conn_info", default_x         , default_y+30   , "Disconnected");

        
        #serial port find val
        self.create_label(None              , default_x+260     , default_y-40   , "GUI OS");
        self.create_combobox("combo0"       , default_x+260     , default_y)
        self.create_button("btcombo0"       , default_x+390     , default_y-5    , "Find"        ,command=self.find_file);

        #motor state info
        self.create_label("motor_state_info", default_x         , default_y+80 , "MOTOR STATE : UNKNOWN");
        
        #motor position info
        self.create_label("lb0"             , default_x      , default_y+100 , "BODY : 0");
        self.create_slider("slide0"         , default_x      , default_y+140, callback=self.slider_info1,from_=60,to=760,length=700);

        self.create_label("lb1"             , default_x      , default_y+190 , "TARGET : 0");
        self.create_slider("slide1"         , default_x      , default_y+230, callback=self.slider_info2,from_=60,to=760,length=700);
        self.create_button("bt1"            , default_x+720  , default_y+225 , "move" ,command=self.usermove);
                


        #motor run val
        self.create_button("bt_RUN"         , default_x+190  , default_y + 75  , "RUN" ,command=self.run_event);
        
        #speed val
        self.create_label("lb2"             , default_x+1100  , default_y+100 , "Speed : 0 m/min");
        self.create_slider("slide2"         , default_x+1130  , default_y+140, callback=self.slider_info3,orient="vertical",tickinterval=1.0,from_=7.0,to=1.0,length=200);

        
        self.create_label(NONE              , default_x          , default_y+300 , "Teaching Point");
        
        self.create_label(NONE              , default_x          , default_y+325 , "#1");
        self.create_entry("ent_tp_0"        , default_x          , default_y+350 , "120",width=10);
        
        self.create_label(NONE              , default_x  +100       , default_y+325 , "#2");
        self.create_entry("ent_tp_1"        , default_x  +100       , default_y+350 , "240",width=10);
    
        self.create_label(NONE              , default_x  +200       , default_y+325 , "#3");
        self.create_entry("ent_tp_2"        , default_x  +200       , default_y+350 , "360",width=10);
    
        self.create_label(NONE              , default_x  +300       , default_y+325 , "#4");
        self.create_entry("ent_tp_3"        , default_x  +300       , default_y+350 , "480",width=10);
    
        self.create_label(NONE              , default_x  +400       , default_y+325 , "#5");
        self.create_entry("ent_tp_4"        , default_x  +400       , default_y+350 , "600",width=10);
    
        self.create_label(NONE              , default_x  +500       , default_y+325 , "#6");
        self.create_entry("ent_tp_5"        , default_x  +500       , default_y+350 , "720",width=10);
    
        self.create_label(NONE              , default_x  +600       , default_y+325 , "#7");
        self.create_entry("ent_tp_6"        , default_x  +600       , default_y+350 , "60",width=10);

        self.create_button("tp_Once"         , default_x+720        , default_y +350  , "Once" ,command=self.once_event);

        self.create_button("tp_Loop"         , default_x+800        , default_y +350  , "Loop" ,command=self.loop_event);
    
        self.create_button("tp_Stop"         , default_x+880        , default_y +350  , "Stop" ,command=self.stop_event);
    
    def set_config(self, serial_name = "", speed = 7,tp_list=[]):
        self.set_entry("ent0",serial_name);
        self.set_slider("slide2",speed);
        [self.set_entry("ent_tp_"+str(idx),tp_) for idx, tp_ in enumerate(tp_list)];
    
    def get_config(self):
        serial_name = self.get_entry("ent0");
        speed=self.get_slider("slide2");
        tp_list = []
        for i in range(7):
            tp_list.append(int(self.get_entry("ent_tp_"+str(i))));
        return serial_name,speed,tp_list



    def once_event(self):
        self.tp_list = [];
        self.tp_count = 0;
        for i in range(7):
            self.tp_list.append(int(self.get_entry("ent_tp_"+str(i))));
        print(self.tp_list)
        self.tp_mode = 0;
        self.tp_live = True;
        self.tk.after(100,self.tp_event)

    def loop_event(self):
        self.tp_list = [];
        self.tp_count = 0;
        for i in range(7):
            self.tp_list.append(int(self.get_entry("ent_tp_"+str(i))));
        self.tp_mode = 1;
        self.tp_live = True;
        self.tk.after(100,self.tp_event)

    def stop_event(self):
        self.tp_live = False;
        self.tp_mode = 0;
    
    def tp_event(self):
        if(self.tp_list.__len__()>0):
            pos=self.tp_list[self.tp_count%self.tp_list.__len__()];
            
            if(self.usermovecheck()==True):
                self.set_slider("slide1", pos);
                self.usermove();
                self.tp_count += 1;
            
            if((self.tp_mode==0) and (self.tp_count == self.tp_list.__len__())):
                self.tp_live =False

        if(self.tp_live==True):
            self.tk.after(500,self.tp_event)

    def find_file(self):
        import glob
        os_name = self.get_combobox("combo0");
        if(os_name in "Linux"):
            folder = '/dev/'
            file_list = glob.glob(f"{folder}/ttyACM*")
            if(file_list.__len__()==0):
                self.set_entry("ent0","NOT FOUND");
            else:
                self.set_entry("ent0",file_list[0]);
        elif(os_name in "MAC"):
            folder = '/dev/'
            file_list = glob.glob(f"{folder}/cu.usbmodem*")
            if(file_list.__len__()==0):
                self.set_entry("ent0","NOT FOUND");
            else:
                self.set_entry("ent0",file_list[0]);
        elif(os_name in "Windows"):
            self.set_entry("ent0",self.windows_serial_port_list[self.windows_serial_port_idx%self.windows_serial_port_list.__len__()]);
            self.windows_serial_port_idx +=1;
        else:
            self.set_entry("ent0","NOT FOUND");



    def update(self):
        
        self.count += 1;
        if(self.count>760):
            self.count = 1
        
        if(not self.motor is None):
            if(self.motor.ser.is_open):
                state,pos=self.motor.get_pos();
                self.set_label("motor_state_info","MOTOR STATE : "+self.MotorStateToStr(state));
                if(state==5):
                    self.set_slider("slide0",int(pos));


        self.tk.after(100,self.update);
    
    def usermovecheck(self):
        res = False;
        if(not self.motor is None):
            if(self.motor.ser.is_open):
                p_state,p_to_pos,p_to_vel,p_to_acc = self.motor.get_profile();
                if(p_state==0):
                    res = True;
        return res;

    def usermove(self):
        res = False;
        if(not self.motor is None):
            if(self.motor.ser.is_open):
                state,pos=self.motor.get_pos();
                p_state,p_to_pos,p_to_vel,p_to_acc = self.motor.get_profile();
                if(p_state==0):
                    speed = int(float(self.get_slider("slide2"))*1000.0/60.0);
                    #acc   = int(speed*2)
                    acc = 300;
                    res = self.motor.set_profile(int(self.get_slider("slide1")),speed,acc);
                    self.tk.after(10,lambda : self.motor.set_move());
                    res = True;
                else:
                    print("not good");
        return res;
    
    def on_closing(self):
        if(not self.motor is None):
            if(self.motor.ser.is_open):
                self.motor.close();
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.tk.destroy()


        
    def slider_info1(self,scale):
        value="BODY : "+str(scale)
        self.set_label("lb0",value);
    
    def slider_info2(self,scale):
        value="TARGET : "+str(scale)
        self.set_label("lb1",value);
    
    def slider_info3(self,scale):
        value="Speed : "+str(scale) + "m/min"
        self.set_label("lb2",value);

    def connect(self):
        if(self.motor is None):
            try:
                self.motor=DKKO_STEPMOTOR_DRIVER( self.get_entry("ent0"));
                res = self.motor.connect();
                print(res);
                if("SNUAILAB" in res):
                    self.set_label("serial_conn_info","Connneted")
                else:
                    if(not self.motor is None):
                        self.motor.close();
                        self.motor = None;
                    self.set_label("serial_conn_info","Mismatching Device")
            except Exception as inst:
                print(inst)
                if(not self.motor is None):
                    self.motor.close();
                    self.motor = None;
                if("No such file or directory" in inst.__str__()):
                    self.set_label("serial_conn_info","not found your device : "+self.get_entry("ent0"))
                else:
                    self.set_label("serial_conn_info","Disconneted")
        
            self.tk.after(100,self.update)


    def save_event(self):
        sn,speed,tplist=self.get_config();
        config = {"SERIALNAME":sn,
                  "SPEED":speed,
                  "TP_LIST":tplist}
        with open('motor_config.yaml', 'w') as f:
            yaml.dump(config, f)

    def run_event(self):
        if(not self.motor is None):
            if(self.motor.ser.is_open):
                state,pos=self.motor.get_pos();
                if(state==0):
                    self.motor.init();



    def load_event(self):
        path = filedialog.askopenfilename(filetypes=[('Config File','.yaml'),('Image File','bmp')])
        
        if path == '':return
        
        if '.yaml' in path:
            yaml_cfg = None;
            with open(path) as f:
                yaml_cfg = yaml.load(f, Loader=yaml.FullLoader);
                sn = yaml_cfg["SERIALNAME"]
                speed = yaml_cfg["SPEED"]
                tplist=  yaml_cfg["TP_LIST"];
                self.set_config(sn,speed,tplist);



    def run(self):
        self.tk.mainloop();
    

    def create_label(self,name = "lab0",pos_x = 0,pos_y = 0,text = "DVD-013",font=None):
        if(not font is None):
            label = Label(self.tk,text=text,font=font);
        else:
            label = Label(self.tk,text=text);
        label.pack();
        label.place(x=pos_x,  y=pos_y);
        if(not name is None):
            self.gui_list[name]=label;
    
    def create_entry(self,name = "ent0",pos_x = 0,pos_y = 0,text = "DVD-013",width=10):
        ent = Entry(self.tk, width=width);
        ent.insert(0,text);
        ent.pack();
        ent.place(x=pos_x, y=pos_y);
        if(not name is None):
            self.gui_list[name]=ent;

    def create_button(self,name = "bt0",pos_x = 0,pos_y = 0,text = "DVD-013",command=None):
        bt = Button(self.tk, text=text,command=command);
        bt.pack()
        bt.place(x=pos_x,  y=pos_y)
        if(not name is None):
            self.gui_list[name]=bt;
    
    def get_entry(self,name = "ent0"):
        if(not name is None):
            return self.gui_list[name].get();
        return "";

    def set_entry(self,name = "ent0",text=""):
        if(not name is None):
            if name in self.gui_list:
                self.gui_list[name].delete(0,'end');
                self.gui_list[name].insert(0,text);
            

    def set_label(self,name = "lab0",text = "DVD-013"):
        if(not name is None):
            if name in self.gui_list:
                self.gui_list[name]['text'] = text;
    
    def create_slider(self,name = "slide0",pos_x = 0,pos_y = 0,callback=None,orient="horizontal",tickinterval=50, from_=0, to=500, length=300):
        var=IntVar()
        scale=Scale(self.tk, variable=var, command=callback, orient=orient, showvalue=False, tickinterval=tickinterval,from_ =from_, to=to, length=length)
        scale.pack();
        scale.place(x=pos_x, y=pos_y,);
        if(not name is None):
            self.gui_list[name]=scale;
    
    def set_slider(self,name="",value=0):
        if(not name is None):
            if name in self.gui_list:
                self.gui_list[name].set(value);
    
    def get_slider(self,name="",value=0):
        if(not name is None):
            if name in self.gui_list:
                return self.gui_list[name].get();
    def create_combobox(self,name="combo0",pos_x = 0,pos_y = 0,values=["MAC", "Linux", "Windows", "Other"],width=12):
        combo = ttk.Combobox(state="readonly",values=values,width=width)
        combo.current(0);
        combo.place(x=pos_x, y=pos_y)
        if(not name is None):
            self.gui_list[name]=combo;
    def get_combobox(self,name="combo0"):
        if(not name is None):
            if name in self.gui_list:
                return self.gui_list[name].get();

    def MotorStateToStr(self,state = None):
        if(state is None):
            return "UNKNOWN"
        elif(state == 0):
            return "IDLE"
        elif(state == 1):
            return "INIT"
        elif(state == 2):
            return "ESCAPE"
        elif(state == 3):
            return "SAFETY"
        elif(state == 4):
            return "PRERUN"
        elif(state == 5):
            return "RUN"
        else:
            return "UNKNOWN"


if __name__ == "__main__":
    gui = gap_inspection_main_gui();
    gui.run();