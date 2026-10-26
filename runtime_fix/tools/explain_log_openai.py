#!/usr/bin/env python3
"""Optional offline diagnosis of an existing log. NOT part of motion control or safety."""
import os,json,sys,urllib.request,urllib.error
from pathlib import Path

if __name__=='__main__':
    if len(sys.argv)!=2:
        sys.exit('Usage: python3 tools/explain_log_openai.py path/to/log.txt')
    token=os.environ.get('OPENAI_API_KEY')
    if not token:sys.exit('Set OPENAI_API_KEY in the terminal; no key is saved to project files.')
    data=Path(sys.argv[1]).read_text(encoding='utf-8',errors='replace')[-24000:]
    payload={'model':os.environ.get('OPENAI_MODEL','gpt-5-mini'),
      'instructions':'Analyze the supplied ROS2/Gazebo diagnostic log. Quote evidence, separate observed facts from hypotheses, and never advise bypassing collision safety. Do not claim live access.',
      'input':data,'store':False}
    request=urllib.request.Request('https://api.openai.com/v1/responses',
          data=json.dumps(payload).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'},method='POST')
    try:
        with urllib.request.urlopen(request,timeout=45) as r:obj=json.load(r)
        outputs=[]
        for item in obj.get('output',[]):
            if item.get('type')=='message':
                outputs.extend(t.get('text','') for t in item.get('content',[]) if t.get('type')=='output_text')
        print('\n'.join(outputs) or json.dumps(obj,ensure_ascii=False,indent=2))
    except urllib.error.HTTPError as e:
        print('API error',e.code,e.read().decode()[:1000],file=sys.stderr);sys.exit(2)
