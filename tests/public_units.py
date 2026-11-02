import csv,io,json,math,subprocess,tempfile
from pathlib import Path
import sys
b=Path(sys.argv[1])
with tempfile.TemporaryDirectory() as d:
 p=Path(d)
 capture=subprocess.check_output([str(b/'guanid_callback_capture')],text=True)
 rows=list(csv.DictReader(io.StringIO(capture)))
 rows += [dict(r, motor='linear', units='m') for r in rows]
 with (p/'si.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
 for r in rows:
  scale=.0254 if r['units']=='m' else math.pi/180
  r['position']=str(float(r['position'])/scale)
  r['velocity']=str(float(r['velocity'])/scale)
  r['units']='in' if r['units']=='m' else 'deg'
 with (p/'public.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)
 def analyze(name,mode,extra=()):
  dest=p/(name+mode+'.json')
  subprocess.run([str(b/'guanid_analyze'),str(p/name),'--units',mode,'--out',str(dest),'--velocity-tau','.1','--position-omega','8','--plot-dir',str(p/'plots'),*extra],check=True,stdout=subprocess.DEVNULL)
  return json.loads(dest.read_text())
 old=analyze('si.csv','si'); new=analyze('si.csv','in-deg'); converted=analyze('public.csv','in-deg')
 assert old['schema_version']==2 and new['schema_version']==3
 for name,ch in old['channels'].items():
  scale=.0254 if ch['position_unit']=='m' else math.pi/180
  out=new['channels'][name]
  assert out['position_unit'] in ('in','deg')
  for key in ('kS','kG','kV','kA'):
   expected=ch['fit'][key]*(scale if key in ('kV','kA') else 1)
   assert math.isclose(expected,out['fit'][key],rel_tol=1e-9,abs_tol=1e-9)
   assert math.isclose(out['fit'][key],converted['channels'][name]['fit'][key],rel_tol=1e-9,abs_tol=1e-9)
  assert ch['fit']['samples']==out['fit']['samples']
  assert math.isclose(ch['fit']['voltage_rmse'],out['fit']['voltage_rmse'],abs_tol=1e-12)
  assert math.isclose(ch['velocity_feedback']['kP']*scale,out['velocity_feedback']['kP'],rel_tol=1e-9)
  assert (out['position_unit']+'/s') in (p/'plots'/f'{name}.svg').read_text()
 tiny=analyze('si.csv','in-deg',('--min-velocity','1e-9'))
 assert tiny['settings']['min_velocity']==1e-9
 assert all(ch['min_velocity']==1e-9 for ch in tiny['channels'].values())
 # Width flags must derive the same physical angular fit from paired wheel records.
 paired=[]
 for row in csv.DictReader((p/'si.csv').open()):
  if row['motor']!='linear': continue
  right=dict(row,motor='right'); left=dict(row,motor='left')
  for key in ('position','velocity','voltage'): left[key]=str(-float(left[key]))
  paired.extend((left,right))
 with (p/'turn.csv').open('w') as f:
  writer=csv.DictWriter(f,fieldnames=paired[0]);writer.writeheader();writer.writerows(paired)
 inches=analyze('turn.csv','in-deg',('--angular-inches','left','right','turn','13','--motor','turn'))
 metres=analyze('turn.csv','in-deg',('--angular','left','right','turn','.3302','--motor','turn'))
 for key in ('kS','kG','kV','kA'):
  assert math.isclose(inches['channels']['turn']['fit'][key],metres['channels']['turn']['fit'][key],abs_tol=1e-10)
 assert inches['channels']['turn']['position_unit']=='deg'
 print('PASS: SI/public CSV; schema 2/3; gains, metrics, plots, threshold precision and angular-width compatibility')

