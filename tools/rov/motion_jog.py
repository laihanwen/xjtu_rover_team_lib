"""Bounded single-axis requests; physical permission remains in the ROV bridge."""
class MotionJog:
    CHANNELS = {'yaw+':(1,159), 'yaw-':(1,95),
                'surge+':(2,159), 'surge-':(2,95),
                'heave+':(3,159), 'heave-':(3,95),
                'sway+':(4,95), 'sway-':(4,159)}

    def __init__(self):
        self.axis = None
        self.started = self.refreshed = 0.
        self.latched = False
        self.reason = '尚未点动'
        self.elapsed = 0.

    def release(self):
        if self.axis is not None:self.reason = '操作者松手 / 页面停止'
        self.axis = None
        self.latched = False

    def request(self, axis, now):
        if axis not in self.CHANNELS:
            raise ValueError('invalid axis')
        if self.latched or (self.axis is not None and self.axis != axis):
            raise ValueError('release before next jog')
        if self.axis is None:
            self.axis, self.started = axis, now
            self.reason = '点动中'
            self.elapsed = 0.
        self.refreshed = now

    def frame(self, now, permitted):
        frame = bytearray([0xa5,127,127,127,127,127,0,2,0,0,0])
        expired = self.axis is not None and (not permitted or now-self.refreshed >= .2 or now-self.started >= 1.)
        if self.axis is not None:self.elapsed = now-self.started
        if expired:
            self.reason = '许可或遥测失效' if not permitted else ('网页续期超时' if now-self.refreshed >= .2 else '达到 1 秒上限')
            self.axis = None
            self.latched = True
        if self.axis is not None:
            channel, value = self.CHANNELS[self.axis]
            frame[channel] = value
        return bytes(frame), expired
