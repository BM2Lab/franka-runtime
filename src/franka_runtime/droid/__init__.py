"""DROID executor/service and offline native-backend candidates.

The public CLI remains mock-only. Imports open no devices/sockets, start no
threads and import no optional vendor libraries. Physical selection/qualification
is intentionally separate from dependency injection used by offline tests.
"""
