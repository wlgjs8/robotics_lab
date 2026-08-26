# Camera Synchronization Redirect

Camera capture and synchronization are owned by `camera_server`, outside the
servo process. Current contracts:

- [../../camera_server/docs/architecture.md](../../camera_server/docs/architecture.md)
- [../../camera_server/docs/sync_policy.md](../../camera_server/docs/sync_policy.md)
- [../../camera_server/docs/metadata_protocol.md](../../camera_server/docs/metadata_protocol.md)
- [../../docs/runbooks/camera_acceptance.md](../../docs/runbooks/camera_acceptance.md)

This redirect replaces an early implementation plan that predated the current
consumer-specific bundle groups and shared-memory transport.
