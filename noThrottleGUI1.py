import tkinter as tk
from tkinter import ttk

class EBikeConfigTool(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("E-bike Configuration Tool")
        self.geometry("900x700")
        
        # Tabs
        self.tab_control = ttk.Notebook(self)
        self.basic_tab = ttk.Frame(self.tab_control)
        self.pedal_assist_tab = ttk.Frame(self.tab_control)
        self.throttle_handle_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.basic_tab, text='Basic')
        self.tab_control.add(self.pedal_assist_tab, text='Pedal Assist')
        self.tab_control.add(self.throttle_handle_tab, text='Throttle Handle')
        self.tab_control.pack(expand=1, fill='both')
        
        # Basic Tab
        self.create_basic_tab()
        
        # Pedal Assist Tab
        self.create_pedal_assist_tab()
        
        # Throttle Handle Tab
        # (Implement similarly if needed)

        # Communication Interface
        self.comm_label = ttk.Label(self, text="Communication Interface")
        self.comm_label.place(x=450, y=20)
        self.com_port_label = ttk.Label(self, text="COM Port:")
        self.com_port_label.place(x=450, y=50)
        self.com_port_combobox = ttk.Combobox(self, values=["COM1", "COM2", "COM3"])
        self.com_port_combobox.place(x=520, y=50)
        self.connect_button = ttk.Button(self, text="Connect", command=self.connect_to_device)
        self.connect_button.place(x=450, y=80)
        self.disconnect_button = ttk.Button(self, text="Disconnect", state=tk.DISABLED)
        self.disconnect_button.place(x=530, y=80)
        
        # Controller Info
        self.controller_info_label = ttk.Label(self, text="Controller Info")
        self.controller_info_label.place(x=450, y=120)
        self.info_labels = ["Manufacturer:", "Model:", "Hardware Ver.:", "Firmware Ver.:", "Nominal Voltage:", "Max. Current:"]
        for i, label in enumerate(self.info_labels):
            ttk.Label(self, text=label).place(x=450, y=150 + i*30)
            ttk.Label(self, text="").place(x=550, y=150 + i*30)
        
        # Buttons for flash
        self.read_flash_button = ttk.Button(self, text="Read Flash")
        self.read_flash_button.place(x=450, y=320)
        self.write_flash_button = ttk.Button(self, text="Write Flash")
        self.write_flash_button.place(x=550, y=320)
        
    def create_basic_tab(self):
        # Low Battery Protection
        ttk.Label(self.basic_tab, text="Low Battery Protection [V]:").grid(column=0, row=0, padx=10, pady=5, sticky=tk.W)
        self.low_battery_combobox = ttk.Combobox(self.basic_tab, values=[i for i in range(20, 51)])
        self.low_battery_combobox.grid(column=1, row=0, padx=10, pady=5, sticky=tk.W)
        
        # Current Limit
        ttk.Label(self.basic_tab, text="Current Limit [A]:").grid(column=0, row=1, padx=10, pady=5, sticky=tk.W)
        self.current_limit_spinbox = ttk.Spinbox(self.basic_tab, from_=0, to=30)
        self.current_limit_spinbox.grid(column=1, row=1, padx=10, pady=5, sticky=tk.W)

    def create_pedal_assist_tab(self):
        # Assist levels labels
        ttk.Label(self.pedal_assist_tab, text="Assist levels").grid(column=0, row=0, padx=10, pady=5, sticky=tk.W)
        ttk.Label(self.pedal_assist_tab, text="Current Limit [%]").grid(column=1, row=0, padx=10, pady=5, sticky=tk.W)
        ttk.Label(self.pedal_assist_tab, text="Speed Limit [%]").grid(column=2, row=0, padx=10, pady=5, sticky=tk.W)
        
        # Assist levels
        self.assist_levels = [("Assist 0:", 10, 10), ("Assist 1:", 20, 20), ("Assist 2:", 30, 30),
                              ("Assist 3:", 40, 40), ("Assist 4:", 50, 50), ("Assist 5:", 60, 60),
                              ("Assist 6:", 70, 70), ("Assist 7:", 80, 80), ("Assist 8:", 90, 90),
                              ("Assist 9:", 100, 100)]
        self.current_limits = []
        self.speed_limits = []
        for i, (label_text, curr_limit, speed_limit) in enumerate(self.assist_levels):
            ttk.Label(self.pedal_assist_tab, text=label_text).grid(column=0, row=i+1, padx=10, pady=5, sticky=tk.W)
            curr_spinbox = ttk.Spinbox(self.pedal_assist_tab, from_=0, to=100, increment=10)
            curr_spinbox.set(curr_limit)
            curr_spinbox.grid(column=1, row=i+1, padx=10, pady=5, sticky=tk.W)
            self.current_limits.append(curr_spinbox)
            speed_spinbox = ttk.Spinbox(self.pedal_assist_tab, from_=0, to=100, increment=10)
            speed_spinbox.set(speed_limit)
            speed_spinbox.grid(column=2, row=i+1, padx=10, pady=5, sticky=tk.W)
            self.speed_limits.append(speed_spinbox)
        
        # Speed Meter Type
        ttk.Label(self.pedal_assist_tab, text="Speed Meter Type:").grid(column=0, row=11, padx=10, pady=5, sticky=tk.W)
        self.speed_meter_combobox = ttk.Combobox(self.pedal_assist_tab, values=["Internal, Wheel Meter", "External, Wheel Meter"])
        self.speed_meter_combobox.grid(column=1, row=11, padx=10, pady=5, sticky=tk.W)
        
        # Speed Meter Signals
        ttk.Label(self.pedal_assist_tab, text="Speed Meter Signals:").grid(column=0, row=12, padx=10, pady=5, sticky=tk.W)
        self.speed_meter_signals_spinbox = ttk.Spinbox(self.pedal_assist_tab, from_=1, to=10)
        self.speed_meter_signals_spinbox.grid(column=1, row=12, padx=10, pady=5, sticky=tk.W)
        
        # Wheel Diameter
        ttk.Label(self.pedal_assist_tab, text="Wheel Diameter [Inch]:").grid(column=0, row=13, padx=10, pady=5, sticky=tk.W)
        self.wheel_diameter_spinbox = ttk.Spinbox(self.pedal_assist_tab, from_=10, to=30)
        self.wheel_diameter_spinbox.grid(column=1, row=13, padx=10, pady=5, sticky=tk.W)
        
        # Read/Write buttons
        self.read_button = ttk.Button(self.pedal_assist_tab, text="READ", command=self.read_parameters)
        self.read_button.grid(column=0, row=14, padx=10, pady=20)
        self.write_button = ttk.Button(self.pedal_assist_tab, text="WRITE", command=self.write_parameters)
        self.write_button.grid(column=1, row=14, padx=10, pady=20)

    def connect_to_device(self):
        print("Connect to device functionality not implemented yet.")
        
    def read_parameters(self):
        print("Read parameters functionality not implemented yet.")
        
    def write_parameters(self):
        print("Write parameters functionality not implemented yet.")

if __name__ == "__main__":
    app = EBikeConfigTool()
    app.mainloop()
