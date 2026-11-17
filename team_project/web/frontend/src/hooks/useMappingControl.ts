import { useCallback, useEffect, useRef, useState } from 'react'
import { toApiError } from '../api/client'
import { adoptSavedMap, getMappingStatus, saveMap, sendMappingCommand, type MappingCommand } from '../api/mappingApi'
import { useStoreData } from '../state/storeDataContext'
import { RESTART_NOTICE } from '../utils/robotControls'
import { useToast } from '../state/toastContext'
import type { MapSaveBody, MapSaveResult, MappingPhase } from '../types/provisional'

const SUCCESS_MESSAGE: Record<MappingCommand, string> = {
  start: '매핑을 시작했습니다.',
  stop: '매핑을 중지했습니다. 지금까지 만든 지도로 초안을 만듭니다.',
  discard: '임시 지도를 버렸습니다.',
}

const POLL_MS = 2000

export { RESTART_NOTICE }

export interface Adoption {
  /** 새 지도가 매장 지도로 반영되었다 */
  adopted: boolean
  needsRestart: boolean
  /** 저장은 됐지만 반영이 보류된 이유 */
  pendingReason: string | null
  navigationValidated: boolean
  validationReason: string | null
  adoptionAllowed: boolean
}

/**
 * 매핑 실행, 검토본 저장, 검사 통과 후 별도 운영 승인.
 * - 요청 중에는 모든 매핑 버튼 비활성 (중복 클릭 방지, ref 로 동기 차단)
 * - 상태는 서버 응답으로만 갱신 (ROS 를 직접 호출하지 않음)
 * - 매핑 중에는 2초마다 상태를 다시 읽어 진행 단계를 보여 주고, 스스로 끝난 것을 알아챈다
 * - onAdopted: 새 지도가 반영된 뒤 화면의 지도·편집 데이터를 다시 읽는 일 (페이지가 넘긴다)
 */
export function useMappingControl(options: { onAdopted?: () => Promise<void> | void } = {}) {
  const { storeId, mappingStatus, setMappingStatus } = useStoreData()
  const { notify } = useToast()
  const [pending, setPending] = useState<MappingCommand | 'save' | 'adopt' | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [adoption, setAdoption] = useState<Adoption | null>(null)
  const inFlight = useRef(false)
  const onAdopted = useRef(options.onAdopted)
  useEffect(() => {
    onAdopted.current = options.onAdopted
  })

  const status = mappingStatus.data
  const phase: MappingPhase | null = status?.phase ?? null
  const draftReady = status?.review_available === true || status?.draft_ready === true

  // 매핑 중 상태 폴링 + 스스로 끝났을 때 알림
  const lastPhase = useRef<MappingPhase | null>(null)
  useEffect(() => {
    if (lastPhase.current === 'mapping' && phase === 'stopped') {
      if (draftReady) notify('지도 초안이 준비되었습니다. 지도를 확인하고 저장하세요.', 'success')
      else if (status?.error) notify(status.error, 'error')
    }
    lastPhase.current = phase
  }, [phase, draftReady, status?.error, notify])

  useEffect(() => {
    if (phase !== 'mapping') return
    let alive = true
    const id = setInterval(() => {
      if (inFlight.current) return
      void getMappingStatus(storeId)
        .then((next) => alive && setMappingStatus(next))
        .catch(() => undefined) // 일시적인 조회 실패는 다음 주기에 다시 시도
    }, POLL_MS)
    return () => {
      alive = false
      clearInterval(id)
    }
  }, [phase, storeId, setMappingStatus])

  const run = useCallback(
    async (command: MappingCommand) => {
      if (inFlight.current) return
      inFlight.current = true
      setPending(command)
      setError(null)
      try {
        const next = await sendMappingCommand(command, storeId)
        setMappingStatus(next)
        if (command === 'start') setAdoption(null)
        notify(SUCCESS_MESSAGE[command], 'success')
      } catch (e) {
        const message = toApiError(e).message
        setError(message)
        notify(message, 'error')
      } finally {
        inFlight.current = false
        setPending(null)
      }
    },
    [storeId, setMappingStatus, notify],
  )

  const afterSave = useCallback(
    async (result: MapSaveResult) => {
      setMappingStatus(result)
      if (result.adopted) {
        setAdoption({ adopted: true, needsRestart: result.needs_robot_restart !== false,
          pendingReason: null, navigationValidated: true, validationReason: null,
          adoptionAllowed: false })
        try {
          await onAdopted.current?.()
        } finally {
          notify('새 지도를 매장 지도로 반영했습니다.', 'success')
        }
      } else {
        const validation = result.review_result?.navigation_validation
        const navigationValidated = result.review_result?.navigation_validated === true
        const handoff = result.review_result?.navigation_handoff
        const verified = (value: unknown) => typeof value === 'object' && value !== null
          && (value as { verified?: unknown }).verified === true
        const autonomyReady = !!handoff
          && verified(handoff.destination_profile)
          && verified(handoff.nav2_route_validation)
        const handoffBlocked = navigationValidated && !autonomyReady
        const reason = result.adopt_error?.message
          ?? (result.review_result
            ? navigationValidated
              ? handoffBlocked
                ? '자율주행 목적지·접근 방향과 Nav2 경로 검증이 끝날 때까지 검토본으로 보관합니다.'
                : '검토본 검사를 통과했습니다. 운영 적용은 별도 승인 후 진행하세요.'
              : `검토본이 저장됐지만 검사를 통과하지 못했습니다: ${validation?.reason ?? '검사 결과 없음'}`
            : '운영 지도 반영이 보류되었습니다.')
        setAdoption({ adopted: false, needsRestart: false, pendingReason: reason,
          navigationValidated, validationReason: validation?.reason ?? null,
          adoptionAllowed: navigationValidated && autonomyReady && result.adopt_error == null })
        notify(reason, 'info')
      }
      return result
    },
    [setMappingStatus, notify],
  )

  /** MapReviewPanel의 검토본 저장 */
  const save = useCallback(
    async (body: MapSaveBody) => {
      if (inFlight.current) throw new Error('다른 매핑 요청을 처리 중입니다.')
      inFlight.current = true
      setPending('save')
      setError(null)
      try {
        return await afterSave(await saveMap(body))
      } finally {
        inFlight.current = false
        setPending(null)
      }
    },
    [afterSave],
  )

  /** 검사를 통과한 검토본을 운영 지도에 별도 승인 후 적용 */
  const adopt = useCallback(async () => {
    if (inFlight.current) return
    if (!adoption?.navigationValidated || !adoption.adoptionAllowed) {
      notify(adoption?.pendingReason ?? '운영 적용에 필요한 검증이 남아 있습니다.', 'error')
      return
    }
    if (!window.confirm('검사에 통과한 지도를 운영 지도에 적용할까요? 적용 전 기존 지도는 백업됩니다.')) return
    inFlight.current = true
    setPending('adopt')
    setError(null)
    try {
      await afterSave(await adoptSavedMap(true))
    } catch (e) {
      const message = toApiError(e).message
      setError(message)
      notify(message, 'error')
    } finally {
      inFlight.current = false
      setPending(null)
    }
  }, [adoption, afterSave, notify])

  const enabled = status?.enabled !== false
  return {
    phase,
    status,
    statusError: mappingStatus.error,
    pending,
    error,
    adoption,
    draftReady,
    enabled,
    canStart: !pending && phase !== null && phase !== 'mapping' && enabled && !draftReady,
    canStop: !pending && phase === 'mapping' && status?.stop_requested !== true,
    canDiscard: !pending && phase !== 'mapping' && draftReady,
    start: () => run('start'),
    stop: () => run('stop'),
    discard: () => run('discard'),
    save,
    adopt,
  }
}

export type MappingControl = ReturnType<typeof useMappingControl>
