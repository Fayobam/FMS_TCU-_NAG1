"""Sample registry entries for the offline preview; these never seed firmware."""
def extend(params):
    def add(idx,id,name,group,lo,hi,values,unit,rows=1):
        params.append(dict(idx=idx,id=id,name=name,group=group,kind=2 if rows>1 else 1 if len(values)>1 else 0,
            min=lo,max=hi,rows=rows,cols=len(values)//rows,unit=unit,v=values,
            help='Offline sample values. The live interface reads this parameter and its help text from the controller.'))
    add(1,'inertia.target','Inertia duration target','Shift feel',120,900,[450,450,440,430,400,350,320,260,220,200,200],'ms')
    for idx,id,name,lo,hi,value,unit in [(2,'apply.floor','Apply pressure floor',0,100,52,'%'),(3,'apply.slope','Apply pressure slope',0,200,90,'x0.01'),
        (4,'inertia.slope','Inertia ramp base',0,800,200,'x0.01'),(5,'inertia.slope.load','Inertia ramp load gain',0,200,20,'x0.001'),
        (6,'backstop.hot','Phase backstop (hot)',300,3000,700,'ms'),(7,'backstop.cold','Phase backstop (cold)',300,4000,1400,'ms')]:
        add(idx,id,name,'Shift feel',lo,hi,[value],unit)
    up=next(p for p in params if p['id']=='auto.up')
    add(9,'auto.dn','Downshift speeds','Auto schedule',2,250,[max(2,v-5) for v in up['v']],'km/h',4)
    for idx,id,name,lo,hi,values,unit in [(10,'firmness','Firmness',50,200,[100,105,115,120,135],'x0.01'),
        (11,'shiftpt','Shift-point scale',50,200,[85,100,120,100,100],'x0.01'),(12,'tccopen','TCC open above TPS',0,100,[60,45,30,30,25],'%'),
        (13,'launch','Launch gear',1,2,[1]*5,'gear'),(14,'auto','Auto shifting',0,1,[1,1,1,0,0],'0/1'),
        (15,'lug','Lug protection',0,1,[1,1,1,1,0],'0/1'),(16,'tqcut','Request torque cut',0,1,[0,0,0,0,1],'0/1')]:
        add(idx,'mode.'+id,name,'Drive modes',lo,hi,values,unit)
    params.sort(key=lambda x:x['idx'])
    return params
