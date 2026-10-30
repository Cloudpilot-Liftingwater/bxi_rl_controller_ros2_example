"""Deferred virtual suspension release, using the existing BXI reset service."""

NORMAL = 'com.bxi.basic_actions/normal'


class SimulationRelease:
    def __init__(self):
        self.normal_since = None
        self.future = None
        self.confirmed = False
        self.failed = False
        self.requested_at = None

    def tick(self, node, now, request_factory):
        if self.confirmed or self.failed:
            return
        if node.topic_prefix != 'simulation/':
            raise ValueError('deferred suspension release is simulation-only')
        if self.future is not None:
            if not self.future.done() and now-self.requested_at>5:
                self.failed=True
                node.get_logger().error('simulation suspension release acknowledgement timed out')
                return
            if self.future.done():
                try:
                    response = self.future.result()
                    if response is None or not response.is_success:
                        raise RuntimeError('simulation reset rejected release')
                except Exception as exc:
                    self.failed = True
                    node.get_logger().error(str(exc))
                else:
                    self.confirmed = True
                    node.get_logger().info('SIM2SIM normal active; BXI virtual suspension release acknowledged')
            return
        framework = node.runtime.framework
        if (framework.current_state_name != NORMAL or framework.state_machine.in_transition
                or framework.loop_count < 1 or not node._imu_received):
            self.normal_since = None
            return
        if self.normal_since is None:
            self.normal_since = now
        if now-self.normal_since < .5 or not node.rest_srv.service_is_ready():
            return
        request = request_factory()
        request.header.frame_id = 'elf3'
        request.reset_step = 2
        request.release = True
        self.future = node.rest_srv.call_async(request)
        self.requested_at = now
