import { expect, it } from 'vitest'
import { parseSemanticMap } from '../api/parsers'
import { createEmptySemanticMap, toSemanticPayload } from './semantic'
import { createEditorState, editorReducer } from './semanticEditor'
import type { MapPoint } from '../types/contracts'

it('운영 구역의 오목한 경계를 편집·저장·재수신해도 유지한다', () => {
  let s = createEditorState(createEmptySemanticMap('demo_store', 1))
  s = editorReducer(s, { type: 'set_tool', tool: 'zone_operating' })
  const polygon: MapPoint[] = [[0, 0], [4, 0], [4, 1], [1, 1], [1, 4], [0, 4]]
  for (const point of polygon) s = editorReducer(s, { type: 'zone_add_point', point })
  s = editorReducer(s, { type: 'zone_finish' })
  const received = parseSemanticMap(toSemanticPayload(s.doc))
  expect(received.zones).toHaveLength(1)
  expect(received.zones[0].type).toBe('operating')
  expect(received.zones[0].polygon).toEqual(polygon)
})

it('같은 테이블의 두 번째 접근점을 선택해 수정하면 첫 번째 접근점은 보존한다', () => {
  const doc = createEmptySemanticMap('demo_store', 1)
  doc.entities = [{ id: 'table_01', type: 'table', label: 'Table 1', pose: { x: 2, y: 2, yaw: 0 } }]
  doc.waypoints = [
    { id: 'wp_table_01_top', entity_id: 'table_01', pose: { x: 2, y: 3, yaw: -Math.PI / 2 } },
    { id: 'wp_table_01_bottom', entity_id: 'table_01', pose: { x: 2, y: 1, yaw: Math.PI / 2 } },
  ]
  const s = editorReducer(createEditorState(doc), {
    type: 'set_waypoint', entityId: 'table_01', waypointId: 'wp_table_01_bottom', pose: { x: 2.1, y: 1, yaw: Math.PI / 2 },
  })
  expect(s.doc.waypoints[0]).toEqual(doc.waypoints[0])
  expect(s.doc.waypoints[1].pose.x).toBe(2.1)
  expect(s.selection).toEqual({ kind: 'waypoint', id: 'wp_table_01_bottom' })
})
