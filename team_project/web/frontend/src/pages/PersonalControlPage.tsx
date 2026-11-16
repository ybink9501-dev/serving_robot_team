import { useEffect, useRef, useState } from 'react'
import { apiClient, request, toApiError } from '../api/client'
import { FleetMap } from '../components/robots/FleetMap'

type Task = { task_id: string; request_id: string; mode: string; destination_id: string;
  robot_id: string | null; status: string; submission: string; mission_phase: string | null;
  cancel_requested?: boolean; detail?: string;
  signals?: { action: string; submission: string; reply: { error?: string } | null }[] }
type Robot = { robot_id: string; mission_phase: string; telemetry: string; request_id: string | null }
type View = { ready: boolean; error: string | null; site_id: string; tasks: Task[]; robots: Robot[];
  controls?: { key: string; submission: string; reply: { error?: string } | null }[];
  fleet: { destinations?: string[]; supervisors?: Record<string, { confirmation_pose_valid?: boolean }> } }
type Draft = { type: 'SERVE'; mode: string; destination_id: string; client_request_id: string }
const draftKey = 'personal-order-draft-v1'
const terminal = ['SUCCEEDED', 'FAILED', 'CANCELED']
const labels: Record<string, string> = { KITCHEN_PICKUP: '주방 적재 후 배달', DELIVER_DIRECT: '직접 배달', TABLE_COLLECT: '테이블 회수' }

export function PersonalControlPage() {
  const [view, setView] = useState<View | null>(null)
  const [error, setError] = useState('')
  const [message, setMessage] = useState('')
  const [mode, setMode] = useState('KITCHEN_PICKUP')
  const [table, setTable] = useState('')
  const [draft, setDraft] = useState<Draft | null>(() => {
    try { return JSON.parse(sessionStorage.getItem(draftKey) ?? 'null') as Draft | null } catch { return null }
  })
  const busy = useRef(false)
  const [pending, setPending] = useState(false)
  const [online, setOnline] = useState(false)
  const refresh = async () => {
    const next = await request(apiClient.get<View>('/api/personal/status'))
    setView(next); setOnline(true)
  }
  useEffect(() => {
    let live = true
    const poll = async () => {
      try {
        const next = await request(apiClient.get<View>('/api/personal/status'))
        if (live) { setView(next); setOnline(true) }
      } catch (e) { if (live) { setOnline(false); setError(toApiError(e).message) } }
    }
    void poll()
    const timer = setInterval(() => void poll(), 1000)
    return () => { live = false; clearInterval(timer) }
  }, [])
  const run = async (call: () => Promise<void>) => {
    if (busy.current) return
    busy.current = true; setPending(true); setError(''); setMessage('')
    try { await call(); await refresh() }
    catch (e) { setError(toApiError(e).message) }
    finally { busy.current = false; setPending(false) }
  }
  const submit = () => run(async () => {
    const body: Draft = draft ?? { type: 'SERVE', mode, destination_id: table, client_request_id: crypto.randomUUID() }
    sessionStorage.setItem(draftKey, JSON.stringify(body)); setDraft(body)
    const task = await request(apiClient.post<Task>('/api/tasks', body))
    setMessage(`${task.task_id}: 저장됨 (${task.submission}). 실제 임무 완료는 상태에서 확인하세요.`)
    sessionStorage.removeItem(draftKey); setDraft(null)
  })
  const signal = (task: Task, action: string, source: string) => run(async () => {
    const reply = await request(apiClient.post<{ submission: string }>('/api/tasks/' + task.task_id + '/signals', { action, source }))
    setMessage(`${task.task_id}: 확인 전달 ${reply.submission}`)
  })
  const home = (robot: string) => run(async () => {
    const key = 'personal-home-' + robot
    const id = sessionStorage.getItem(key) ?? crypto.randomUUID()
    sessionStorage.setItem(key, id)
    const reply = await request(apiClient.post<{ submission: string }>(`/api/robots/${robot}/home`, { client_request_id: id }))
    setMessage(`${robot}: 복귀 전달 ${reply.submission}`)
    sessionStorage.removeItem(key)
  })
  const ready = online && view?.ready === true
  return <div className="page">
    <h1>개인 Fleet 관제</h1>
    <p>주문·이력은 Backend, 실제 배차·주행·복귀는 개인 Fleet가 담당합니다.</p>
    <p role="status">{ready ? `연결됨 · ${view?.site_id}` : `조작 대기 · ${view?.error ?? '연결 확인 중'}`}</p>
    {error && <p role="alert">{error}</p>}{message && <p role="status">{message}</p>}
    <section className="card" style={{ padding: 16 }}>
      <h2>임무 접수</h2>
      <label>임무 종류 <select value={mode} disabled={pending || !!draft} onChange={e => setMode(e.target.value)}>
        {Object.entries(labels).map(([id, label]) => <option key={id} value={id}>{label}</option>)}
      </select></label>
      <label> 테이블 <select value={table} disabled={pending || !!draft} onChange={e => setTable(e.target.value)}>
        <option value="">선택</option>{view?.fleet.destinations?.map(id => <option key={id} value={id}>{id}</option>)}
      </select></label>
      {draft && <p>미확인 요청: {labels[draft.mode]} / {draft.destination_id}. 같은 ID로 재시도합니다.</p>}
      <button disabled={!ready || pending || (!draft && !table)} onClick={() => void submit()}>{draft ? '동일 요청 재시도' : '임무 접수'}</button>
    </section>
    <section className="card" style={{ minHeight: 420, display: 'flex', marginTop: 16 }}><FleetMap /></section>
    <h2>로봇</h2>
    {view?.robots.map(robot => <section className="card" key={robot.robot_id} style={{ padding: 12 }}>
      <strong>{robot.robot_id}</strong> · {robot.mission_phase} · {robot.telemetry} · {robot.request_id ?? '임무 없음'}
      <button disabled={!ready || pending || !['IDLE', 'COMPLETED'].includes(robot.mission_phase)} onClick={() => void home(robot.robot_id)}>HOME 복귀 요청</button>
    </section>)}
    <h2>주문·확인</h2>
    {view?.controls?.map(control => <p key={control.key}>복귀 요청: {control.submission} {control.reply?.error}</p>)}
    {view?.tasks.slice().reverse().map(task => {
      const confirm = task.mission_phase === 'WAIT_LOAD' ? ['LOAD', 'kitchen', '주방 적재 확인'] :
        task.mission_phase === 'WAIT_HANDOVER' ? ['COMPLETE', 'table', '테이블 수령 확인'] :
        task.mission_phase === 'WAIT_COLLECTION' ? ['COLLECTED', 'table', '테이블 회수 확인'] : null
      return <section className="card" key={task.task_id} style={{ padding: 12 }}>
        <p>{task.task_id} · {labels[task.mode]} · {task.destination_id} · {task.robot_id ?? '배정 대기'}</p>
        <p>전달 {task.submission} / 임무 {task.status} / 단계 {task.mission_phase ?? '—'} {task.cancel_requested && '/ 취소 확인 대기'}</p>
        {task.detail && <p role="alert">{task.detail}</p>}
        {task.signals?.map(signal => <p key={signal.action}>{signal.action}: {signal.submission} {signal.reply?.error}</p>)}
        {confirm && <button disabled={!ready || pending || !view?.robots.some(r => r.robot_id === task.robot_id && r.telemetry === 'FRESH') || !view?.fleet.supervisors?.[task.robot_id ?? '']?.confirmation_pose_valid}
          onClick={() => void signal(task, confirm[0], confirm[1])}>{confirm[2]} ({task.task_id})</button>}
        {!terminal.includes(task.status) && <button disabled={!ready || pending || task.cancel_requested}
          onClick={() => void run(async () => { await request(apiClient.post('/api/tasks/' + task.task_id + '/cancel')); setMessage('취소 요청 저장. 실제 종료를 기다립니다.') })}>취소 요청</button>}
      </section>
    })}
  </div>
}
