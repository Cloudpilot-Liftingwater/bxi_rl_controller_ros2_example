"""Optional passive evaluator. Observation failures never change control flow."""
from weakref import WeakSet

_command_listeners = WeakSet()


def add_command_listener(listener):
    _command_listeners.add(listener)


def remove_command_listener(listener):
    _command_listeners.discard(listener)

try:
    from elf3_eval import host as _host
except Exception:
    _host = None


def observe(method, *args, **kwargs):
    if method == "command_published":
        for listener in tuple(_command_listeners):
            try:
                listener.command_published(*args, **kwargs)
            except Exception:
                listener.observer_failed = True
    if _host is None:
        return
    try:
        getattr(_host, method)(*args, **kwargs)
    except Exception:
        # Includes an older installed evaluator without the dance bridge.
        pass


def log_message(logger, severity, message):
    """Use stable Humble call sites; a log sink must not interrupt control.

    rclpy caches severity per Python call site. A dynamic getattr(...)(...)
    at one line cannot alternate INFO/WARN/ERROR on a real ROS logger.
    Lifecycle events are separately retained by the passive observer.
    """
    try:
        if severity == "error":
            logger.error(message)
        elif severity == "warning":
            logger.warning(message)
        else:
            logger.info(message)
    except Exception:
        return False
    return True
