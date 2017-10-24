
'''
Interface for a PS4 Controller
Jonathan
'''


# Buttons
# value: 0, 1
# 0 released, 1 down
X         = 0
CIRCLE    = 1
TRIANGLE  = 2
SQUARE    = 3
L1        = 4
R1        = 5
L2        = 6
R2        = 7
SHARE     = 8
OPTIONS   = 9
HOME      = 10 #Also center pad
L3        = 11
R3        = 12

# Axes
# value: from -1 to 1
# Analog (X) -> 1 is right, -1 is left
# Analog (Y) -> 1 is down, -1 is up
# Triggers   -> -1 is released, 1 is down, 0 is centered
LX = 0
LY = 1
LT = 2
RX = 3
RY = 4
RT = 5
  
# DPad
# value: -1, 0, 1
# 0 is released
# DX -> 1 is right, -1 is left
# DY -> 1 is up, -1 is down    
DX = 0
DY = 1

#PS4Controller Class
class PS4Controller(object):

    def __init__(self,config=0):
        self._config = config
        
        self._buttons = [0 for i in range(13)]
        self._axes = [0 for i in range(6)]
        self._dpad = [0 for i in range(2)]
    
    
    def create(self):
        # Controller Module
        self._cm = PygameModule(self._config)
        #self._cm = JoystickModule()
        
        created = self._cm.create()
        if not created:
            return False
        
        return True
        
        
    
    # Polls input, should be called every frame/update
    def poll(self):
        self._buttons, self._axes, self._dpad = self._cm.poll()
        
    # Calls cm quit
    def destroy(self):
        self._cm.destroy()
    
    
    # Buttons
    
    def x(self): return self._buttons[X]
    def circle(self): return self._buttons[CIRCLE]
    def triangle(self): return self._buttons[TRIANGLE]
    def square(self): return self._buttons[SQUARE]
    
    def L1(self): return self._buttons[L1]
    def R1(self): return self._buttons[R1]
    def L2(self): return self._buttons[L2]
    def R2(self): return self._buttons[R2]
    
    def options(self): return self._buttons[OPTIONS]
    def share(self): return self._buttons[SHARE]
    def home(self): return self._buttons[HOME]
    
    def L3(self): return self._buttons[L3]
    def R3(self): return self._buttons[R3]
    
    
    # Axes
    
    def LX(self): return self._axes[LX]
    def LY(self): return self._axes[LY]
    def LT(self): return self._axes[LT]
    
    def RX(self): return self._axes[RX]
    def RY(self): return self._axes[RY]
    def RT(self): return self._axes[RT]
    
    
    # Dpad
    
    def DX(self): return self._dpad[DX]
    def DY(self): return self._dpad[DY]
    
    def up(self): return 1 if self._dpad[DY] == 1 else 0
    def down(self): return 1 if self._dpad[DX] == -1 else 0
    def right(self): return 1 if self._dpad[DY] == 1 else 0
    def left(self): return 1 if self._dpad[DX] == -1 else 0








#Controller Module Interface
class ControllerModule(object):

    # Creates controller module, returns True if created, False if failed
    def create(self):
        pass
    
    # Return (buttons, axes, dpad) for PS4Controller
    def poll(self):
        pass
    
    # Destroys controller module
    def destroy(self):
        pass






import joystick

class JoystickModule(ControllerModule):
    
    def __init__(self):
        pass
        
    def create(self):
        self._joystick = joystick.Joystick()
        
        created = self._joystick.create()
        if not created:
            return False
        
        self._joystick.start()
        
        return True
        
    def poll(self):
        buttons = self._joystick.getButtons()
        axes = self._joystick.getAxis()
        
        buttons = [0 for i in range(13)]
        axes = [0 for i in range(6)]
        dpad = [0 for i in range(2)]
        return buttons, axes, dpad
        
    def destroy(self):
        self._joystick.destroy()

    







import pygame

#Pygame Module

def config0(buttons,axes,hats):
    return buttons, axes, hats

def config1(buttons,axes,hats):
    nbuttons = [i for i in range(13)]
    nbuttons[X]         = buttons[1]
    nbuttons[CIRCLE]    = buttons[2]
    nbuttons[TRIANGLE]  = buttons[3]
    nbuttons[SQUARE]    = buttons[0]
    nbuttons[L1]        = buttons[4]
    nbuttons[R1]        = buttons[5]
    nbuttons[L2]        = buttons[6]
    nbuttons[R2]        = buttons[7]
    nbuttons[OPTIONS]   = buttons[8]
    nbuttons[SHARE]     = buttons[9]
    nbuttons[HOME]      = 1 if buttons[12]==1 or buttons[13]==1 else 0
    nbuttons[L3]        = buttons[11]
    nbuttons[R3]        = buttons[10]
    naxes = [i for i in range(6)]
    naxes[LX] = 0
    naxes[LY] = 1
    naxes[LT] = 3
    naxes[RX] = 2
    naxes[RY] = 5
    naxes[RT] = 4
    return nbuttons, naxes, hats

class PygameModule(ControllerModule):

    def __init__(self, config=0):
        if config == 0:
            self._config = config0
        elif config == 1:
            self._config = config1
        
    def create(self):
        pygame.init()
        count = pygame.joystick.get_count()
        if count == 0:
            return False
        
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        
        self._lt_pressed = False
        self._rt_pressed = False
        return True
        
    def poll(self):
        pygame.event.pump()
        buttons = [self._joystick.get_button(i) for i in range(self._joystick.get_numbuttons())]
        axes = [self._joystick.get_axis(i) for i in range(self._joystick.get_numaxes())]
        hats_array = [self._joystick.get_hat(i) for i in range(self._joystick.get_numhats())]
        hats = [hats_array[0][0], hats_array[0][1]]
        buttons, axes, hats = self._config(buttons, axes, hats)
        self._handle_triggers(axes)
        return buttons, axes, hats
    
    # For pygame joysticks ( for or PS4 controllers in general), the left and right trigger values
    # are set to 0.0 even though they are -1.0 at rest (not pressed).
    # After pressing a trigger for the first time, the correct value is set but otherwise,
    # it stays at 0.0
    #
    # This function handles setting the trigger values to -1.0 by default until another value
    # is detected.
    def _handle_triggers(self,axes):
        if self._lt_pressed == False:
            if axes[2] != 0.0:
                self._lt_pressed = True
            else:
                axes[2] = -1.0
        if self._rt_pressed == False:
            if axes[5] != 0.0:
                self._rt_pressed = True
            else:
                axes[5] = -1.0
        
    def destroy(self):
        self._joystick.quit()
        pygame.quit()



    




