import { fireEvent, render, screen, waitFor, cleanup } from '@testing-library/react'
import { afterEach, beforeEach, expect, test, vi } from 'vitest'
import { PersonalControlPage } from './PersonalControlPage'

const api = vi.hoisted(() => ({ get: vi.fn(), post: vi.fn() }))
vi.mock('../api/client', () => ({ apiClient: api,
  request: async (promise: Promise<{ data: unknown }>) => (await promise).data,
  toApiError: (e: unknown) => e instanceof Error ? e : new Error(String(e)) }))
vi.mock('../components/robots/FleetMap', () => ({ FleetMap: () => <div>개인 지도</div> }))

const snapshot = { ready: true, error: null, site_id: 'test-site', tasks: [], robots: [],
  fleet: { destinations: ['table_03'], supervisors: {} } }
beforeEach(() => { sessionStorage.clear(); api.get.mockReset(); api.post.mockReset(); api.get.mockResolvedValue({ data: snapshot }) })
afterEach(() => { cleanup(); sessionStorage.clear() })

test('response loss retries the identical order id and explicit mission mode', async () => {
  api.post.mockRejectedValueOnce(new Error('응답 유실')).mockResolvedValueOnce({ data: { task_id: 'task-one', submission: 'PENDING' } })
  render(<PersonalControlPage />)
  await screen.findByText('연결됨 · test-site')
  fireEvent.change(screen.getByLabelText('테이블'), { target: { value: 'table_03' } })
  fireEvent.click(screen.getByRole('button', { name: '임무 접수' }))
  await screen.findByText('응답 유실')
  fireEvent.click(screen.getByRole('button', { name: '동일 요청 재시도' }))
  await waitFor(() => expect(api.post).toHaveBeenCalledTimes(2))
  const first = api.post.mock.calls[0][1]; const second = api.post.mock.calls[1][1]
  expect(second).toEqual(first)
  expect(first.mode).toBe('KITCHEN_PICKUP')
  expect(first.client_request_id).toBeTruthy()
  await screen.findByText(/실제 임무 완료는 상태에서 확인하세요/)
})

test('handover and collection keep different signals and request paths', async () => {
  api.get.mockResolvedValue({ data: { ...snapshot,
    tasks: [{ task_id: 'delivery', mode: 'DELIVER_DIRECT', destination_id: 'table_03', robot_id: 'robot_01', status: 'RUNNING', submission: 'ACCEPTED', mission_phase: 'WAIT_HANDOVER' },
            { task_id: 'collection', mode: 'TABLE_COLLECT', destination_id: 'table_03', robot_id: 'robot_02', status: 'RUNNING', submission: 'ACCEPTED', mission_phase: 'WAIT_COLLECTION' }],
    robots: [{ robot_id: 'robot_01', mission_phase: 'WAIT_HANDOVER', telemetry: 'FRESH' }, { robot_id: 'robot_02', mission_phase: 'WAIT_COLLECTION', telemetry: 'FRESH' }],
    fleet: { destinations: ['table_03'], supervisors: { robot_01: { confirmation_pose_valid: true }, robot_02: { confirmation_pose_valid: true } } } } })
  api.post.mockResolvedValue({ data: { submission: 'PENDING' } })
  render(<PersonalControlPage />)
  fireEvent.click(await screen.findByRole('button', { name: '테이블 수령 확인 (delivery)' }))
  await waitFor(() => expect(api.post).toHaveBeenCalledWith('/api/tasks/delivery/signals', { action: 'COMPLETE', source: 'table' }))
  await waitFor(() => expect(screen.getByRole('button', { name: '테이블 회수 확인 (collection)' })).toBeEnabled())
  fireEvent.click(screen.getByRole('button', { name: '테이블 회수 확인 (collection)' }))
  await waitFor(() => expect(api.post).toHaveBeenCalledWith('/api/tasks/collection/signals', { action: 'COLLECTED', source: 'table' }))
  expect(screen.getAllByRole('button', { name: 'HOME 복귀 요청' }).every(button => button.hasAttribute('disabled'))).toBe(true)
})
