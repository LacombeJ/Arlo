
'''
Interface for a PS4 Controller
Jonathan
'''


import pygame


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


#Pygame Module
class PygameModule(ControllerModule):

    def __init__(self):
        pygame.init()
        
    def create(self):
        count = pygame.joystick.get_count()
        if count == 0:
            return False
        
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        return True
        
    def poll(self):
        pygame.event.pump()
        buttons = [self._joystick.get_button(i) for i in range(self._joystick.get_numbuttons())]
        axes = [self._joystick.get_axis(i) for i in range(self._joystick.get_numaxes())]
        hats_array = [self._joystick.get_hat(i) for i in range(self._joystick.get_numhats())]
        hats = [hats_array[0][0], hats_array[0][1]]
        return (buttons, axes, hats)
        
    def destroy(self):
        pygame.quit()


#PS4Controller Class
class PS4Controller(object):

    def __init__(self):
        # init buttons
        # value: 0, 1
        # 0 released, 1 down
        self._X         = 0
        self._CIRCLE    = 1
        self._TRIANGLE  = 2
        self._SQUARE    = 3
        self._L1        = 4
        self._R1        = 5
        self._L2        = 6
        self._R2        = 7
        self._OPTIONS   = 8
        self._SHARE     = 9
        self._HOME      = 10 #Also center pad
        self._L3        = 11
        self._R3        = 12
        self._buttons = [0 for i in range(13)]
        
        # init axes
        # value: from -1 to 1
        # Analog (X) -> 1 is right, -1 is left
        # Analog (Y) -> 1 is down, -1 is up
        # Triggers   -> -1 is released, 1 is down, 0 is centered
        self._LX = 0
        self._LY = 1
        self._LT = 2
        self._RX = 3
        self._RY = 4
        self._RT = 5
        self._axes = [0 for i in range(6)]
        
        # init dpad
        # value: -1, 0, 1
        # 0 is released
        # DX -> 1 is right, -1 is left
        # DY -> 1 is up, -1 is down
        self._DX = 0
        self._DY = 1
        self._dpad = [0 for i in range(2)]
    
    
    def create(self):
        # Controller Module
        self._cm = PygameModule()
        
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
    
    def x(self): return self._buttons[self._X]
    def circle(self): return self._buttons[self._CIRCLE]
    def triangle(self): return self._buttons[self._TRIANGLE]
    def square(self): return self._buttons[self._SQUARE]
    
    def L1(self): return self._buttons[self._L1]
    def R1(self): return self._buttons[self._R1]
    def L2(self): return self._buttons[self._L2]
    def R2(self): return self._buttons[self._R2]
    
    def options(self): return self._buttons[self._OPTIONS]
    def share(self): return self._buttons[self._SHARE]
    def home(self): return self._buttons[self._HOME]
    
    def L3(self): return self._buttons[self._L3]
    def R3(self): return self._buttons[self._R3]
    
    
    # Axes
    
    def LX(self): return self._axes[self._LX]
    def LY(self): return self._axes[self._LY]
    def LT(self): return self._axes[self._LT]
    
    def RX(self): return self._axes[self._RX]
    def RY(self): return self._axes[self._RY]
    def RT(self): return self._axes[self._RT]
    
    
    # Dpad
    
    def DX(self): return self._dpad[self._DX]
    def DY(self): return self._dpad[self._DY]
    
    def up(self): return 1 if self._dpad[self._DY] == 1 else 0
    def down(self): return 1 if self._dpad[self._DX] == -1 else 0
    def right(self): return 1 if self._dpad[self._DY] == 1 else 0
    def left(self): return 1 if self._dpad[self._DX] == -1 else 0
    




