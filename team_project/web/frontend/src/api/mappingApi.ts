import type { MappingCommandBody } from '../types/contracts'
import type { MapReviewData, MapSaveBody, MapSaveResult, MappingStatus } from '../types/provisional'
import { apiClient, request } from './client'

export type MappingCommand = 'start' | 'stop' | 'discard'

/**
 * POST /api/mapping/{start|stop|discard}  body: { store_id }
 * 응답 형식은 임시 계약(MappingStatus). Backend 확정 시 여기서만 변환한다.
 */
export function sendMappingCommand(command: MappingCommand, storeId: string): Promise<MappingStatus> {
  const body: MappingCommandBody = { store_id: storeId }
  return request(apiClient.post<MappingStatus>(`/api/mapping/${command}`, body))
}

/** [임시 계약] GET /api/mapping/status?store_id= */
export function getMappingStatus(storeId: string): Promise<MappingStatus> {
  return request(apiClient.get<MappingStatus>('/api/mapping/status', { params: { store_id: storeId } }))
}

/** 검토본 저장 및 완성 검사. 운영 지도 반영은 별도 승인 API로 수행 */
export function saveMap(body: MapSaveBody): Promise<MapSaveResult> {
  return request(apiClient.post<MapSaveResult>('/api/mapping/save', body))
}

/** 저장은 됐지만 반영이 보류된 경우 다시 시도 */
export function adoptSavedMap(confirmed: boolean): Promise<MapSaveResult> {
  return request(apiClient.post<MapSaveResult>('/api/mapping/adopt', { confirmed }))
}

/** 보정 화면 정보 (초안이 준비된 뒤에만) */
export function getMapReview(storeId: string): Promise<MapReviewData> {
  return request(apiClient.get<MapReviewData>(`/api/stores/${encodeURIComponent(storeId)}/map-review`))
}
