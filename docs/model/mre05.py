import cadquery as cq
AX=5.26   # axis height above holder bottom
def cyl(r,x0,x1,z=AX):
    return cq.Workplane("YZ").workplane(offset=x0).center(0,z).circle(r).extrude(x1-x0)
# axial leads (55.4 long)
leads=cyl(0.325,-27.7,27.7)
# glass reed capsule + coil flange ends
glass=cyl(1.5,-9.8,-6.5).union(cyl(1.5,6.5,10.6))
flange=cyl(2.1,-9.2,-8.5).union(cyl(2.1,7.5,8.2))
# yellow coil body
body=cyl(3.1,-5.7,5.7)
# holder: base plate 28 x 8 x 2.4
base=cq.Workplane("XY").box(28,8,2.4,centered=(True,True,False))
# end walls 8 wide, 9.2 tall, 0.8 thick, chamfered top, notch on top
def wall(xc):
    w=(cq.Workplane("YZ").workplane(offset=xc-0.4)
       .polyline([(-4,0),(4,0),(4,6.5),(3,9.2),(-3,9.2),(-4,6.5)]).close().extrude(0.8))
    w=w.cut(cq.Workplane("XY").box(1,0.9,1.5,centered=(True,True,False))
            .translate((xc,0,7.7)))
    hole=cyl(1.1,xc-1,xc+1)
    return w.cut(hole)
walls=wall(-6.1).union(wall(6.17))
holder=base.union(walls)
# connection pins dia 0.65, 5.04 apart, 3.7 below, offset as drawn
pins=None
for x in (1.24-11.34,1.24+11.34):
    for y in (-2.52,2.52):
        p=cq.Workplane("XY").workplane(offset=-3.7).center(x,y).circle(0.325).extrude(3.7+2.4)
        pins=p if pins is None else pins.union(p)
parts={"holder":(holder,(0.25,0.25,0.27)),"body":(body,(0.95,0.9,0.3)),
       "glass":(glass.union(flange),(0.6,0.85,0.75)),
       "leads":(leads,(0.75,0.75,0.78)),"pins":(pins,(0.75,0.75,0.78))}
asm=cq.Assembly(name="MRE05-1A79-BV384")
for n,(s,c) in parts.items(): asm.add(s,name=n,color=cq.Color(*c))
asm.save("MRE05-1A79-BV384.step")
allp=holder.union(body).union(glass).union(flange).union(leads).union(pins)
cq.exporters.export(allp,"MRE05-1A79-BV384.stl",tolerance=0.01)
bb=allp.val().BoundingBox();print(bb.xlen,bb.ylen,bb.zlen,bb.zmin)
