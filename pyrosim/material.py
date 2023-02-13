import logging

from pyrosim.commonFunctions import Save_Whitespace
from enum import Enum

class COLOR(Enum):
    Cyan = 0
    Green = 1

class MATERIAL: 

    def __init__(self, color):

        self.depth  = 3

        if color is COLOR.Cyan:
            self.string1 = '<material name="Cyan">'
            self.string2 = '    <color rgba="0 1.0 1.0 1.0"/>'
            self.string3 = '</material>'
        elif color is COLOR.Green:
            self.string1 = '<material name="Green">'
            self.string2 = '    <color rgba="0 1.0 0 1.0"/>'
            self.string3 = '</material>'
        else:
            logging.error('invalid color')

    def Save(self,f):

        Save_Whitespace(self.depth,f)

        f.write( self.string1 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string2 + '\n' )

        Save_Whitespace(self.depth,f)

        f.write( self.string3 + '\n' )
