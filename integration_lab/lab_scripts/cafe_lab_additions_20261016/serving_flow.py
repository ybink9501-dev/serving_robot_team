#!/usr/bin/env python3
"""Pure state machine for kitchen -> table -> confirm -> kitchen (no ROS dependency)."""
import time


class ServingFlow:
    def __init__(self, kitchen='kitchen', mode='manual', auto_wait_s=10.0, clock=time.monotonic):
        if mode not in ('manual', 'auto'):
            raise ValueError('mode must be manual or auto')
        if auto_wait_s < 0:
            raise ValueError('auto_wait_s must be nonnegative')
        self.kitchen = kitchen
        self.mode = mode
        self.auto_wait_s = float(auto_wait_s)
        self.clock = clock
        self.state = 'IDLE'
        self.table = None
        self.started_wait = None
        self.reason = ''

    def _status(self, reason=''):
        self.reason = reason
        return ('STATUS', self.state, self.table, reason)

    def start(self, table):
        if not isinstance(table, str) or not table.startswith('table_') or len(table) < 7:
            return [self._status('REJECTED_INVALID_TABLE')]
        if self.state not in ('IDLE', 'COMPLETED', 'CANCELED', 'ERROR'):
            return [self._status('REJECTED_BUSY')]
        self.table = table
        self.started_wait = None
        self.state = 'GOING_KITCHEN'
        return [('GO', self.kitchen), self._status('GO_KITCHEN')]

    def robot_state(self, state, destination_id):
        expected = {
            'GOING_KITCHEN': self.kitchen,
            'GOING_TABLE': self.table,
            'RETURNING_KITCHEN': self.kitchen,
        }.get(self.state)
        if expected is None or destination_id != expected:
            return []
        if state in ('ERROR', 'PAUSED'):
            self.state = 'ERROR'
            return [self._status(f'NAV_{state}')]
        if state != 'SERVING':
            return []
        # DestinationServer emits SERVING only after Nav2 success AND physical arrival.
        if self.state == 'GOING_KITCHEN':
            self.state = 'GOING_TABLE'
            return [('GO', self.table), self._status('GO_TABLE')]
        if self.state == 'GOING_TABLE':
            self.state = 'WAIT_HANDOVER'
            self.started_wait = self.clock()
            return [self._status('WAIT_FOR_MANUAL_OR_AUTO_COMPLETION')]
        if self.state == 'RETURNING_KITCHEN':
            self.state = 'COMPLETED'
            return [self._status('ROUND_TRIP_COMPLETED')]
        return []

    def complete(self):
        if self.state != 'WAIT_HANDOVER':
            return [self._status('REJECTED_NOT_WAITING')]
        self.state = 'RETURNING_KITCHEN'
        self.started_wait = None
        return [('GO', self.kitchen), self._status('RETURN_TO_KITCHEN')]

    def tick(self):
        if self.mode == 'auto' and self.state == 'WAIT_HANDOVER':
            if self.clock() - self.started_wait >= self.auto_wait_s:
                return self.complete()
        return []

    def cancel(self):
        if self.state in ('IDLE', 'COMPLETED', 'CANCELED'):
            return [self._status('NO_ACTIVE_SERVING_TASK')]
        self.state = 'CANCELED'
        self.started_wait = None
        return [('CANCEL',), self._status('CANCEL_REQUESTED')]
