#!/usr/bin/env python3
"""Stub server for ``/request_package``.

Stands in for the real UR7 driver while the arm side of the project is
being built. When the scout arrives at a hazard and calls the service,
this node logs the request, waits a configurable amount of time (to
roughly mimic the arm picking + handing over a package), and returns a
canned package label based on the hazard category.

Swap this out for the actual UR7 node when it exists -- the interface
lives in ``scout_msgs/srv/RequestPackage``.
"""
from __future__ import annotations

import time
from typing import Dict

import rclpy
from rclpy.node import Node

from scout_msgs.srv import RequestPackage


DEFAULT_MAP: Dict[str, str] = {
    'fire':     'mini_extinguisher',
    'chemical': 'neutralizer_kit',
    'medical':  'first_aid_pack',
}


class Ur7Stub(Node):
    def __init__(self):
        super().__init__('ur7_stub')

        self.declare_parameter('prep_seconds', 2.0)
        self.declare_parameter('always_accept', True)

        self.prep = float(self.get_parameter('prep_seconds').value)
        self.always_accept = bool(self.get_parameter('always_accept').value)

        self.srv = self.create_service(
            RequestPackage, '/request_package', self._on_request
        )
        self.get_logger().info(
            f'UR7 stub ready (prep={self.prep:.1f}s, always_accept={self.always_accept})'
        )

    def _on_request(
        self,
        request: RequestPackage.Request,
        response: RequestPackage.Response,
    ) -> RequestPackage.Response:
        self.get_logger().info(
            f'[ur7_stub] request: id={request.hazard_id} cat={request.category} '
            f'at ({request.hazard_position.x:.2f}, {request.hazard_position.y:.2f})'
        )
        label = DEFAULT_MAP.get(request.category, f'generic_{request.category or "package"}')

        # Block for `prep` seconds to simulate arm motion. This is fine
        # because services are already executed on their own thread.
        time.sleep(self.prep)

        response.accepted = self.always_accept
        response.package_label = label
        response.estimated_prep_seconds = float(self.prep)
        response.message = (
            'stubbed UR7; package ready' if self.always_accept
            else 'stub configured to reject'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Ur7Stub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
