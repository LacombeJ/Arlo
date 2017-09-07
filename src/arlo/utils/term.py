
'''
term.py

This module is a utility for printing in different terminal colors and styles

'''


PURPLE      = '\033[95m'
CYAN        = '\033[96m'
DARKCYAN    = '\033[36m'
BLUE        = '\033[94m'
GREEN       = '\033[92m'
YELLOW      = '\033[93m'
RED         = '\033[91m'
BOLD        = '\033[1m'
UNDERLINE   = '\033[4m'
END         = '\033[0m'


def _print2(SA,SB,a,b):
    print SA+a + SB+b + END

def print_blue_white(a,b):
    _print2(BLUE,END,a,b)
    
def print_blue_purple(a,b):
    _print2(BLUE,PURPLE,a,b)
