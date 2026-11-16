"""Reuse TEAM map editor/validator in personal mode; outputs stay provisional."""
import copy,hashlib,json,math,re,threading
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import yaml
from PIL import Image
from fastapi import APIRouter
from .semantics import SemanticStore,SemanticError,validate,_atomic_json,_read_json,digest
from .mapping.session import MappingConfig,MappingSession,MappingError
from .util import now_iso


def runtime_mode(root):
    try:
        record=json.loads((Path(root)/'_local_only/runtime.json').read_text())
        command=Path('/proc/'+str(int(record['pid']))+'/cmdline').read_bytes()
        return record['mode'] if b'team_control.py' in command else None
    except (OSError,ValueError,KeyError):return None


def mapping_motion_active(root):
    if runtime_mode(root)!='mapping':return False
    try:
        import fcntl
        with (Path(root)/'_local_only/mapping_session.lock').open('r') as lock:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:return True
            fcntl.flock(lock,fcntl.LOCK_UN);return False
    except (OSError,ImportError):return False


def active_body(profile,root):
    """Read the active local footprints; never fall back to another site's body."""
    dimensions=[]
    try:
        for robot in profile['robots']:
            p=Path(profile['nav2_files'][robot]);p=p if p.is_absolute() else Path(root)/p
            nav=yaml.safe_load(p.read_text())
            local=next(v for k,v in nav.items() if k=='local_costmap' or k.endswith('/local_costmap'))
            pts=json.loads(local['local_costmap']['ros__parameters']['footprint'])
            if len(pts)<3 or not all(len(p)==2 and all(math.isfinite(float(v)) for v in p) for p in pts):raise ValueError('Invalid footprint')
            shape=(max(float(x) for x,y in pts),-min(float(x) for x,y in pts),max(abs(float(y)) for x,y in pts))
            if min(shape)<=0:raise ValueError('Invalid body dimensions')
            dimensions.append(shape)
        if any(any(abs(a-b)>1e-9 for a,b in zip(d,dimensions[0])) for d in dimensions):
            raise ValueError('Mixed robot footprints require per-robot route validation')
    except (OSError,KeyError,TypeError,ValueError,StopIteration) as exc:
        raise SemanticError(422,'FOOTPRINT_CONTRACT_FAILED',str(exc))
    return dimensions[0]


def seed_semantics(gateway,store):
    entities,waypoints=[],[]
    for name,dest in gateway.destinations.items():
        poses=dest.get('approaches',{'default':dest});coords=list(poses.values())
        kind='table' if name in gateway.tables else 'kitchen' if name=='kitchen' else 'robot_wait'
        center=dict(x=sum(v['x'] for v in coords)/len(coords),y=sum(v['y'] for v in coords)/len(coords),yaw=0.)
        eid=('robot_wait_'+name.split('_')[1]) if kind=='robot_wait' else name
        entities.append(dict(id=eid,type=kind,label=name,pose=center))
        for side,pose in poses.items():waypoints.append(dict(id=f'wp_{eid}_{side}',entity_id=eid,pose=pose,manual=True))
    zones=[];active=yaml.safe_load(Path(gateway.profile['zones_file']).read_text())
    for kind,key in [('exclusive','zones'),('no_go','no_go'),('slow','slow')]:
        for name,zone in (active.get(key,{}) or {}).items():
            b=zone.get('bounds') if isinstance(zone,dict) else zone
            if isinstance(b,(list,tuple)) and len(b)==4:
                zones.append(dict(id=kind+'_'+name,type=kind,polygon=[[b[0],b[2]],[b[1],b[2]],[b[1],b[3]],[b[0],b[3]]]))
    return dict(store_id=store,map_revision=1,entities=entities,waypoints=waypoints,zones=zones)


class PersonalSiteSetup:
    def __init__(self,gateway,settings):
        self.gateway,self.settings=gateway,settings
        db=Path(settings.personal_db) if settings.personal_db and settings.personal_db!=':memory:' else Path(settings.repo)/'_local_only/web_personal/backend.sqlite3'
        self.folder=db.parent/'site_setup';self.lock=threading.RLock()
        self.site=SimpleNamespace(store_id=settings.store_id,revision=1,map_yaml_path=gateway.profile['map_yaml'],semantics=seed_semantics(gateway,settings.store_id))
        self.store=SemanticStore(self.site,settings.repo,str(self.folder))
        self.mapping=MappingSession(MappingConfig(store_id=settings.store_id,work_dir=settings.mapping_work_dir or str(self.folder/'mapping'),
            runner=settings.mapping_runner or str(Path(settings.repo)/'scripts/mapping_session.py'),ros_python=settings.mapping_ros_python,
            enabled=settings.mapping_enabled,require_ros_env=settings.mapping_require_ros_env))

    def context(self,store):
        if store!=self.settings.store_id:raise SemanticError(404,'NO_SEMANTICS','매장을 찾지 못했습니다.')
        from .personal import load_shared
        try:current=load_shared(self.settings.repo,'profile_contract').load_profile(self.settings.repo,self.settings.personal_config)
        except (OSError,ValueError,KeyError,yaml.YAMLError) as exc:
            raise SemanticError(409,'ACTIVE_PROFILE_UNAVAILABLE','현재 지도·프로필을 확인할 수 없습니다: '+str(exc))
        if current['profile_revision']!=self.gateway.profile['profile_revision']:
            raise SemanticError(409,'ACTIVE_PROFILE_CHANGED','지도·운영 프로필이 바뀌었습니다. 현재 프로필로 Backend를 다시 실행하세요.')
        return current['profile_revision']

    def load(self,store):
        context=self.context(store);saved=_read_json(str(self.folder/'context.json'))
        return self.store.load_draft() if saved and saved.get('profile_revision')==context else None

    def save(self,store,doc):
        with self.lock:
            context=self.context(store);saved=self.store.save_draft(doc)
            _atomic_json(str(self.folder/'context.json'),dict(profile_revision=context,frame_id='map',coordinate_transform='identity'))
            return saved

    def graph(self,store,doc=None):
        if mapping_motion_active(self.settings.repo):raise SemanticError(409,'MAPPING_BUSY','매핑 종료 후 경로·좌표 검사를 실행하세요. 설정 초안 저장은 가능합니다.')
        self.context(store);doc=doc or self.load(store) or seed_semantics(self.gateway,store)
        validate(doc,store,1)
        result=self.store.graph(doc)
        result['notes'].append('개인 Fleet는 Nav2로 경로를 계획합니다. 이 미리보기만으로 Gazebo 주행 검증이나 운영 적용을 완료하지 않습니다.')
        owners=[p['entity_id'] for p in doc['waypoints']]
        if len(set(owners))<len(owners):result['notes'].append('기존 그래프 생성기는 객체당 대표 접근점 하나를 표시합니다. 전체 명시 접근점은 별도의 지도·목적지 묶음 검사에서 보존·검사합니다.')
        return result

    def bundle(self,store,doc):
        """All explicit approaches remain in ROS map coordinates; no silent snapping."""
        context=self.context(store);validate(doc,store,1)
        operating=[z for z in doc['zones'] if z['type']=='operating']
        if len(operating)!=1:raise SemanticError(422,'OPERATING_REGION_REQUIRED','운영 구역을 하나 지정하세요.')
        from .personal import load_shared
        import sys
        source=str(Path(self.settings.repo)/'ros_ws/src/serving_robot_mapping')
        if source not in sys.path:sys.path.insert(0,source)
        from serving_robot_mapping.mapping_polygon import validate_polygon,contains,polygon_mask
        try:polygon=validate_polygon(operating[0]['polygon'])
        except (ValueError,TypeError) as exc:raise SemanticError(422,'INVALID_OPERATING_REGION',str(exc))
        profile=self.gateway.profile;m=yaml.safe_load(Path(profile['map_yaml']).read_text())
        pixels=np.flipud(np.array(Image.open(profile['image_path']))).astype(float);h,w=pixels.shape
        res=float(m['resolution']);ox,oy,yaw=map(float,m['origin']);co,si=math.cos(yaw),math.sin(yaw)
        if m.get('mode')=='raw':free=(pixels>=0)&(pixels<=25)
        else:
            occupancy=pixels/255. if m.get('negate',0) else 1.-pixels/255.
            free=occupancy<float(m.get('free_thresh',.25))
        yy,xx=np.mgrid[:h,:w];xx=(xx+.5)*res;yy=(yy+.5)*res
        mx=ox+co*xx-si*yy;my=oy+si*xx+co*yy
        free &= polygon_mask(polygon,mx,my)
        zones={'zones':{},'no_go':{},'slow':{},'retreat':{}}
        for z in doc['zones']:
            if z['type']=='operating':continue
            p=validate_polygon(z['polygon']);xs=[v[0] for v in p];ys=[v[1] for v in p]
            bounds=[min(xs),max(xs),min(ys),max(ys)]
            if len(p)!=4 or set(p)!={(bounds[0],bounds[2]),(bounds[0],bounds[3]),(bounds[1],bounds[2]),(bounds[1],bounds[3])}:
                raise SemanticError(422,'NON_RECTANGULAR_RUNTIME_ZONE','현재 로봇 구역 형식에는 직사각형 금지·감속·전용 구역이 필요합니다. 운영 구역은 오목 다각형도 지원합니다.')
            if z['type']=='no_go':zones['no_go'][z['id']]=bounds;free &= ~polygon_mask(p,mx,my)
            elif z['type']=='slow':
                current=yaml.safe_load(Path(profile['zones_file']).read_text()).get('slow',{}) or {}
                name=z['id'][len('slow_'):] if z['id'].startswith('slow_') else z['id']
                ratio=float(current.get(name,{}).get('speed_ratio',min((float(v.get('speed_ratio',.4)) for v in current.values()),default=.4)))
                zones['slow'][z['id']]=dict(bounds=bounds,speed_ratio=ratio)
            else:zones['zones'][z['id']]=bounds
        limits_path=Path(profile['limits_file'])
        limits_path=limits_path if limits_path.is_absolute() else Path(self.settings.repo)/limits_path
        limits=yaml.safe_load(limits_path.read_text())
        margin=float(limits['dynamic_obstacles']['hard_stop_clearance_m'])
        # Same body and HardStop source as the existing TEAM site generator.
        self.store._site_builder();from site_builder.params import load_params
        body=load_params(self.settings.repo)
        body.front_m,body.rear_m,body.half_width_m=active_body(profile,self.settings.repo)
        radius=math.hypot(max(body.front_m,body.rear_m)+margin,body.half_width_m+margin)
        from site_builder.clearance import clearance_map
        field=clearance_map(~free,res)-res*(1/math.sqrt(2)-.5)
        body_radius=float(limits['dynamic_obstacles']['robot_radius_m'])
        alignment_required=[]
        def check_pose(p,label):
            x,y=float(p['x'])-ox,float(p['y'])-oy
            col,row=math.floor((co*x+si*y)/res),math.floor((-si*x+co*y)/res)
            if not (0<=row<h and 0<=col<w) or field[row,col]<body_radius:
                raise SemanticError(422,'UNSAFE_STOP',label+'의 차체·장애물 여유가 부족합니다.')
            heading=float(p.get('yaw',0.));c,s=math.cos(heading),math.sin(heading)
            corners=[(p['x']+c*x-s*y,p['y']+s*x+c*y) for x,y in
                     [(body.front_m+margin,body.half_width_m+margin),(body.front_m+margin,-body.half_width_m-margin),
                      (-body.rear_m-margin,-body.half_width_m-margin),(-body.rear_m-margin,body.half_width_m+margin)]]
            local=[(co*(x-ox)+si*(y-oy),-si*(x-ox)+co*(y-oy)) for x,y in corners]
            left,right=min(q[0] for q in local),max(q[0] for q in local)
            bottom,top=min(q[1] for q in local),max(q[1] for q in local)
            if left<=0 or bottom<=0 or right>=w*res or top>=h*res:raise SemanticError(422,'UNSAFE_STOP',label+'의 footprint가 지도 밖입니다.')
            # Exact rectangle/square separating-axis test. This tests the requested
            # orientation, not an unnecessary full rotation at every serving stop.
            rows,cols=np.mgrid[max(0,int(bottom/res)):min(h,int(top/res)+1),max(0,int(left/res)):min(w,int(right/res)+1)]
            gx,gy=(cols+.5)*res,(rows+.5)*res
            wx,wy=ox+co*gx-si*gy,oy+si*gx+co*gy
            dx,dy=wx-p['x'],wy-p['y'];rx,ry=c*dx+s*dy,-s*dx+c*dy
            half=res/2*(abs(math.cos(yaw-heading))+abs(math.sin(yaw-heading)))
            hit=((~free[rows,cols])&(gx+res/2>=left)&(gx-res/2<=right)&(gy+res/2>=bottom)&(gy-res/2<=top)
                 &(rx+half>=-body.rear_m-margin)&(rx-half<=body.front_m+margin)
                 &(ry+half>=-body.half_width_m-margin)&(ry-half<=body.half_width_m+margin))
            if hit.any():raise SemanticError(422,'UNSAFE_STOP',label+'의 정차 방향 footprint가 장애물·미확인 영역과 겹칩니다.')
            if field[row,col]<radius:alignment_required.append(label)
            return (row,col)
        entities={e['id']:e for e in doc['entities']};destinations={};components=set();homes=set();kitchens=0;tables=0
        for eid,e in entities.items():
            if e['type']=='counter':continue
            stops=[p for p in doc['waypoints'] if p['entity_id']==eid]
            if not stops:raise SemanticError(422,'MISSING_APPROACH',eid+'의 접근 위치가 없습니다.')
            poses={}
            for p in stops:
                pose=copy.deepcopy(p['pose']);components.add(check_pose(pose,p['id']))
                if e['type']=='table':
                    if not e.get('pose') or not e.get('top_size_m'):raise SemanticError(422,'MISSING_TABLE_GEOMETRY',eid+'의 중심·상판 크기가 필요합니다.')
                    center=e['pose'];dx,dy=pose['x']-center['x'],pose['y']-center['y'];angle=center.get('yaw',0.)
                    lx,ly=math.cos(angle)*dx+math.sin(angle)*dy,-math.sin(angle)*dx+math.cos(angle)*dy
                    normal=(0. if lx<0 else math.pi) if abs(lx)>=abs(ly) else (math.pi/2 if ly<0 else -math.pi/2)
                    error=math.atan2(math.sin(pose['yaw']-angle-normal),math.cos(pose['yaw']-angle-normal))
                    if abs(error)>float(limits['arrival']['control_yaw_target_rad']):raise SemanticError(422,'TABLE_NOT_PERPENDICULAR',p['id']+'의 방향이 테이블 변에 수직이 아닙니다.')
                    axis_x=abs(lx)>=abs(ly)
                    across=abs(ly) if axis_x else abs(lx)
                    half_across=e['top_size_m']['y' if axis_x else 'x']/2
                    half_front=e['top_size_m']['x' if axis_x else 'y']/2
                    gap=(abs(lx) if axis_x else abs(ly))-half_front-body.front_m
                    if across>half_across or not body.gap_min_m<=gap<=body.gap_max_m:
                        raise SemanticError(422,'TABLE_APPROACH_MISSES_FACE',p['id']+'가 상판 변을 바라보지 않거나 앞 간격이 맞지 않습니다.')
                side=p['id'][len('wp_'+eid+'_'):] if p['id'].startswith('wp_'+eid+'_') else p['id']
                poses[side]=pose
            if e['type']=='table':destinations[eid]={'approaches':poses};tables+=1
            elif e['type']=='kitchen':
                kitchens+=1
                if len(poses)!=1:raise SemanticError(422,'AMBIGUOUS_KITCHEN','주방 접근 위치를 하나 지정하세요.')
                destinations['kitchen']=next(iter(poses.values()))
            else:
                match=re.fullmatch(r'(robot_\d+)_HOME',eid) or re.fullmatch(r'robot_wait_(\d+)',eid)
                robot=(match[1] if eid.endswith('_HOME') else 'robot_'+match[1]) if match else None
                if robot not in profile['robots'] or robot in homes or len(poses)!=1:raise SemanticError(422,'HOME_ROBOT_BINDING','대기 위치와 로봇의 연결이 모호합니다.')
                homes.add(robot);destinations[robot+'_HOME']=next(iter(poses.values()));zones['retreat'][robot]=destinations[robot+'_HOME']
        if kitchens!=1 or not tables or homes!=set(profile['robots']):raise SemanticError(422,'INCOMPLETE_DESTINATIONS','주방 하나·테이블·각 로봇 HOME이 필요합니다.')
        for robot,p in profile['robots'].items():components.add(check_pose(dict(x=p['map_x'],y=p['map_y'],yaw=p['map_yaw']),robot+' 초기 위치'))
        from collections import deque
        reachable={next(iter(components))};queue=deque(reachable)
        while queue:
            row,col=queue.popleft()
            for rr,cc in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
                if 0<=rr<h and 0<=cc<w and (rr,cc) not in reachable and field[rr,cc]>=body_radius:
                    reachable.add((rr,cc));queue.append((rr,cc))
        if not components.issubset(reachable):raise SemanticError(422,'DISCONNECTED_DESTINATIONS','접근 위치가 동일한 안전 이동 영역에 연결되지 않습니다.')
        return dict(schema_version=1,frame_id='map',site_id=profile['site_id'],source_profile_revision=context,
                    map_revision=doc['map_revision'],source_semantics_sha=digest(doc),coordinate_transform='identity',
                    map_yaml=profile['map_yaml'],destinations=destinations,zones=zones,operating_polygon=polygon,
                    turn_clearance_radius_m=radius,alignment_required=alignment_required,
                    static_checks_scope='oriented_HardStop_and_body_connected_component',
                    static_checks_passed=True,gazebo_validation='NOT_RUN',navigation_authorized=False)

    def apply(self,store):
        if mapping_motion_active(self.settings.repo):raise SemanticError(409,'MAPPING_BUSY','매핑 종료 후 설정 인계 검사를 실행하세요.')
        with self.lock:
            doc=self.load(store)
            if doc is None:raise SemanticError(409,'NO_DRAFT','먼저 매장 설정을 저장하세요.')
            bundle=self.bundle(store,doc)
            import tempfile,shutil
            parent=self.folder/'bundles';parent.mkdir(parents=True,exist_ok=True)
            final=parent/(bundle['source_profile_revision'][:12]+'_'+digest(doc))
            if final.exists():
                applied=_read_json(str(final/'applied.json'))
                _atomic_json(str(self.folder/'personal_bundle/applied.json'),applied)
                return applied
            out=Path(tempfile.mkdtemp(prefix='staging-',dir=parent))
            for name,value in [('waypoints.yaml',{'destinations':bundle['destinations']}),('zones.yaml',bundle['zones']),
                               ('mapping_boundary.yaml',dict(schema_version=1,frame_id='map',site_id=bundle['site_id'],operator_approved=False,
                                   approval_basis='Frontend draft only; explicit boundary approval still required',interior_polygon=bundle['operating_polygon'],closed_portals=[]))]:
                target=out/name;temporary=target.with_suffix('.tmp');temporary.write_text(yaml.safe_dump(value));temporary.replace(target)
            profile=copy.deepcopy(self.gateway.profile)
            source_meta=yaml.safe_load(Path(profile['map_yaml']).read_text());source_meta['image']='map.pgm'
            (out/'map.yaml').write_text(yaml.safe_dump(source_meta));shutil.copy2(profile['image_path'],out/'map.pgm')
            profile.update(map_yaml=str(final/'map.yaml'),waypoints_file=str(final/'waypoints.yaml'),zones_file=str(final/'zones.yaml'),
                           mapping_boundary_contract=str(final/'mapping_boundary.yaml'),mapping_wall_first=True,
                           mapping_grid_resolution_m=float(source_meta['resolution']),mapping_goal_tolerance_m=.01,
                           mapping_bootstrap_yaw_rad=.18,mapping_planner_clearance_alignment=True,
                           mapping_slam_overlay='_local_only/mapping_slam_285/install/setup.bash')
            profile.pop('profile_revision',None);profile.pop('image_path',None)
            (out/'profile.yaml').write_text(yaml.safe_dump(profile))
            warnings=['운영 지도·로봇 실행은 변경되지 않았습니다.']
            if bundle['alignment_required']:warnings.append('제자리 회전 여유가 부족하여 접근 중 방향 정렬의 Gazebo 검증이 필요합니다: '+', '.join(bundle['alignment_required']))
            summary=dict(OK=len(doc['waypoints']),WARN=len(warnings),FAIL=0)
            applied=dict(applied_at=now_iso(),source_sha=digest(doc),summary=summary,output_dir=str(final),notes=['개인 Fleet용 좌표·방향·연결을 정적 검사했습니다. Gazebo 검증과 운영 전환 승인은 별도입니다.'],warnings=warnings)
            _atomic_json(str(out/'bundle.json'),bundle);_atomic_json(str(out/'applied.json'),applied)
            self.context(store)  # Refuse a map/profile race before publishing the inactive bundle.
            out.replace(final)
            _atomic_json(str(self.folder/'personal_bundle/applied.json'),applied)
            return applied

    def status(self,store):
        self.context(store);draft=self.load(store);applied=_read_json(str(self.folder/'personal_bundle/applied.json'))
        return dict(draft=None if draft is None else self.store.status()['draft'],applied=applied,
                    in_sync=bool(draft and applied and digest(draft)==applied['source_sha']))

    def mapping_status(self):
        result=self.mapping.status();result['saved']=self.mapping.saved
        result['runtime_mode']=runtime_mode(self.settings.repo)
        result['external_session_active']=mapping_motion_active(self.settings.repo) and not self.mapping.active
        result['enabled']=bool(self.settings.mapping_enabled and result['runtime_mode']=='mapping' and not result['external_session_active'])
        return result


def mount_site_setup(app,setup):
    """Same frontend contracts; no second order dispatcher or auto map adoption."""
    from .personal import Problem,TERMINAL
    from fastapi.responses import JSONResponse,Response
    router=APIRouter();store=setup.settings.store_id
    @app.exception_handler(SemanticError)
    @app.exception_handler(MappingError)
    async def setup_error(request,exc):
        return JSONResponse(dict(code=exc.code,message=exc.message,details=exc.details),status_code=exc.status)

    @router.get('/api/runtime/mode')
    def mode():return dict(mode=runtime_mode(setup.settings.repo),supported=['mapping','operation'],exclusive_owner=True)

    @router.put('/api/stores/{sid}/semantics')
    def save(sid:str,body:dict):return setup.save(sid,body)

    @router.get('/api/stores/{sid}/semantics/status')
    def status(sid:str):return setup.status(sid)

    @router.get('/api/stores/{sid}/semantics/graph')
    def graph(sid:str):return setup.graph(sid)

    @router.post('/api/stores/{sid}/semantics/graph')
    def graph_body(sid:str,body:dict):return setup.graph(sid,body)

    @router.post('/api/stores/{sid}/semantics/apply')
    def apply(sid:str):return setup.apply(sid)

    @router.get('/api/stores/{sid}/semantics/live')
    def live(sid:str):setup.context(sid);return dict(matches_applied=False,backup_count=0,last=None)

    @router.get('/api/stores/{sid}/semantics/backups')
    def backups(sid:str):setup.context(sid);return []

    @router.post('/api/stores/{sid}/semantics/promote')
    def promote(sid:str):
        setup.context(sid)
        raise SemanticError(409,'GAZEBO_VALIDATION_REQUIRED','실제 지도·접근 방향·주행 인계 검증 및 별도 운영 전환 승인이 필요합니다. 현재 운영 지도는 유지됩니다.')

    @router.post('/api/mapping/start')
    def mapping_start(body:dict):
        if runtime_mode(setup.settings.repo)!='mapping':raise Problem('MAPPING_MODE_REQUIRED')
        if mapping_motion_active(setup.settings.repo):raise Problem('ALREADY_MAPPING')
        setup.context(store)
        setup.mapping.start(lambda:any(t['status'] not in TERMINAL for t in setup.gateway.view()['tasks']))
        return setup.mapping_status()

    @router.post('/api/mapping/stop')
    def mapping_stop(body:dict):setup.mapping.stop();return setup.mapping_status()

    @router.post('/api/mapping/discard')
    def mapping_discard(body:dict):setup.mapping.discard();return setup.mapping_status()

    @router.get('/api/stores/{sid}/map-review')
    def review(sid:str):setup.context(sid);return setup.mapping.review_service().describe()

    @router.get('/api/stores/{sid}/map-review.png')
    def review_image(sid:str):setup.context(sid);return Response(setup.mapping.review_service().preview_png(),media_type='image/png')

    @router.post('/api/mapping/save')
    def mapping_save(body:dict):
        result=setup.mapping.save_review(body)
        return dict(review_result=result,adopted=False,adopt_error=dict(code='MAP_AND_DESTINATIONS_VALIDATION_REQUIRED',message='초안을 저장했습니다. 목적지·접근 방향·Gazebo 검증 후 운영 전환은 별도 승인합니다.'))

    @router.post('/api/mapping/adopt')
    def mapping_adopt(body:dict):raise Problem('MAP_AND_DESTINATIONS_VALIDATION_REQUIRED')
    app.include_router(router)
