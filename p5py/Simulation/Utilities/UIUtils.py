from ..UIElements.ButtonUtils import Rect

'''
Global : origin is center of bottom screen, x rightwards along width , y upwards along height
Rect : origin is top left of screen , x rightwards along width, y downwards along height

global to rect scale : 
return point in global p5 coordinates.
'''
global2P5Scale = 100
def scalep5ToGlobal(length) : 
    global global2P5Scale
    return length/ global2P5Scale 
 # 1m is 100 pixels 

def scaleGlobalToP5(length):
    global global2P5Scale
    return length * global2P5Scale 

def transformPositionToP5(rect : Rect, worldPosition):
    topLeft = rect.tl
    bottomRight = rect.br
    size = rect.size
    screenPosition = [0,0]
    screenPosition[0] = global2P5Scale* (worldPosition[0])+ (topLeft[0] + bottomRight[0])/2 
    screenPosition[1] = size[1] - (worldPosition[1])*global2P5Scale 
    return screenPosition


'''
Rect : origin is top left of screen , x rightwards along width, y downwards along height
Global : origin is center of bottom screen, x rightwards along width , y upwards along height

return point with bottom left of rect as origin.

rect to global scale : 
'''
def transformPositionToGlobal(rect : Rect , screenPosition):
    topLeft = rect.tl
    bottomRight = rect.br
    size = rect.size
    worldPosition = [0,0]
    worldPosition[0] = (screenPosition[0] - (topLeft[0] + bottomRight[0])/2)/global2P5Scale
    worldPosition[1] = (size[1] - screenPosition[1])/global2P5Scale
    return worldPosition
    