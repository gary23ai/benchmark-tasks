"""Generate the millimetre fabrication profiles and assembly elevations (DXF R2010)."""
from math import sqrt
from pathlib import Path
import ezdxf

out = Path(__file__).resolve().parent
doc = ezdxf.new("R2010")
doc.units = 4
for name, color in (("CUT", 7), ("HOLE", 3), ("ASSEMBLY", 5), ("EXTENDED", 1), ("NOTES", 2)):
    doc.layers.new(name, dxfattribs={"color": color})
m = doc.modelspace()

def line(a,b,layer="ASSEMBLY"):
    m.add_line(a,b,dxfattribs={"layer":layer})

def rect(x,y,w,h,layer="CUT"):
    m.add_lwpolyline([(x,y),(x+w,y),(x+w,y+h),(x,y+h)],close=True,dxfattribs={"layer":layer})

def hole(x,y,d):
    m.add_circle((x,y),d/2,dxfattribs={"layer":"HOLE"})

def label(x,y,s,h=3):
    m.add_text(s,dxfattribs={"height":h,"layer":"NOTES","insert":(x,y)})

def slot(x1,x2,y,r):
    for sign in [-1,1]:
        line((x1,y+r*sign),(x2,y+r*sign),"HOLE")
    m.add_arc((x1,y),r,90,270,dxfattribs={"layer":"HOLE"})
    m.add_arc((x2,y),r,270,90,dxfattribs={"layer":"HOLE"})

def bar(x,y,pitch,width,margin,d):
    rect(x,y,pitch+2*margin,width)
    for pos in [0,pitch/2,pitch]:
        hole(x+margin+pos,y+width/2,d)

label(0,79,"01  FLAT PARTS - mm, 1:1 centerlines, see BUILD.md")
bar(0,50,47,8,4,3.1)
label(0,62,"40 x mast links 2 mm Al; 47 pin pitch")
bar(0,15,435,10,5,4.1)
label(0,31,"2 x rear bar guide links 3 mm Al; 435 pin pitch")
rect(0,-43,60,24)
rect(34,-37,12,12,"HOLE")
for x in (3,7):hole(x,-31,2.2)
label(0,-13,"Tab 4 mm Al, 12 square through window")
rect(0,-94,106,12)
slot(19,99,-88,1.15)
label(0,-69,"Tube axial slot detail (development); 4 x; width 2.3")
rect(0,-139,68,12)
slot(8,58,-133,2.05)
label(0,-115,"Mast shoe track: pivot-center groove 50 x 4.1; see origins")
rect(0,-179,440,12)
slot(6,431,-173,2.55)
label(0,-155,"Bar guide track: 425 x 5.1 center travel")
for Y in [0,525]:
    raised=Y>0
    layer="EXTENDED" if raised else "ASSEMBLY"
    x=620+Y
    label(x-80,481,"02  MAST "+("LATCHED" if raised else "RESET")+"  (Y-Z view, table Z=0)")
    h=424 if raised else 104
    width=sqrt(47**2-(h/10)**2)
    for i in range(10):
        z=18+i*h/10
        for ya,yb in [(0,-width),(-width,0)]:
            line((x+ya,z),(x+yb,z+h/10),layer)
    for i,od in enumerate([32,28,24,20,16]):
        bottom=14+(80*i if raised else 0)
        rect(x-24-od/2,bottom,od,106,layer)
    shoulder=132+(320 if raised else 0)
    rect(x-33,shoulder-4,18,4,layer)
    rect(x-28,shoulder,8,8,layer)
    label(x-90,-15,"W = %.3f, lower fixed at Y=269"%width)
    label(x-90,-22,"Z shoulder %.0f; pin 8 high"%shoulder)
for index,(lift,tag) in enumerate([(0,"RESET"),(305,"SEATED"),(345,"MAX")]):
    x=660+index*510
    base=-690
    label(x-20,base+456,"03  REAR BAR GUIDE "+tag)
    h=70+lift
    spread=sqrt(435**2-h**2)
    for a,b in [((0,18),(spread,18+h)),((spread,18),(0,18+h))]:
        line((x+a[0],base+a[1]),(x+b[0],base+b[1]))
    line((x,base+18),(x+435,base+18))
    line((x,base+18+h),(x+435,base+18+h))
    label(x,base+3,"435 pitch; Y0=-236; dx %.2f"%spread)
label(0,-254,"04  LID plan: horizontal axis Y (-300 at left), vertical axis X (-40 at bottom)")
rect(0,-350,600,80)
rect(92,-313,416,6,"HOLE")
rect(55,-296,458,16,"HOLE")
rect(514,-339,84,58,"HOLE")
label(0,-365,"Openings: cloth; rear scissors; mast. Split overlapping outlines into one opening.")
# The bridge top view uses horizontal X and vertical Y relative to the shoulder.
label(0,-406,"05  UPPER BRIDGE / SHOULDER PLAN - origin at post axis")
bx,by=44,-478
bridge=[(-33,-33),(-14,-33),(-14,-25),(14,-25),(14,-33),(33,-33),(33,29),(13,29),(13,18),(3.5,18),(3.5,8),(-3.5,8),(-3.5,18),(-13,18),(-13,29),(-33,29)]
m.add_lwpolyline([(bx+x,by+y) for x,y in bridge],close=True,dxfattribs={"layer":"CUT"})
hole(bx,by,19)
for dx in (-23,23):
    line((bx+dx,by-31),(bx+dx,by+28))
label(85,-473,"3 thick; track flanges at X=+/-23; notch clears riser.")
label(85,-481,"Rear cutout clears knob and stem; merge slot with Ø19 bore.")
label(0,-545,"06  WINDING / CORD PLAN - axes Y right, X up; vertical levels in BUILD.md")
def plan(y,x): return (300+y,-635+x)
for x in [-40,40]:
    line(plan(-300,x),plan(300,x))
for y in [-300,300]:
    line(plan(y,-40),plan(y,40))
for x in [-29,-1]:
    line(plan(-211,x),plan(211,x))
line(plan(-225,-34),plan(208,-34))
line(plan(-292,-30),plan(195,-30))
line(plan(-100,16),plan(263,16))
line(plan(-100,2),plan(202,2))
hole(*plan(245,0),32)
for y in [-225,-215,215]:
    line(plan(y,-35),plan(y,0))
label(0,-694,"Roller X=-15 Z=52; fabric exits X=0; cord drum at Y=-225.")
label(0,-702,"Parallel extension springs X=-30 Z=30 / 73, anchor Y=-292; turnaround Y=208.")
label(0,-710,"Separate 2:1 post return in base channel X=16 / 2, Z=11; cap eye X=16 Y=270.")
label(0,-748,"07 LATCH - plan schematic; pin axis X through cheek / strike")
rect(185,-863,18,3)
line((194,-863),(194,-882))
rect(189,-883,10,10)
label(213,-864,"Traveling 3-thick strike, Ø3.3 receiver. 2.5-3 engaged.")
label(213,-875,"Fixed bore Ø3.1 x 10; Ø3 pin, 0.4 chamfer.")
label(213,-886,"5 OD spring, 3.5 release travel. Through-wall section below.")
label(0,-926,"08 LOCK SECTION, X horizontal, in engaged position; section origin X=0")
for xpos,width,height in [(30,3,10),(33,10,10),(43,3,6),(46,9,5),(55,3,8)]:
    rect(xpos,-951-height/2,width,height,"ASSEMBLY")
line((30.2,-951),(64,-951))
line((40,-967),(40,-933))
label(72,-949,"Case side at 40. Strike 30..33; guide to 43; collar to 46.")
label(72,-957,"Spring to 55; drilled M8 rear plug to 58; side guard to 66.")
doc.saveas(out/"screen-fixture.dxf")
