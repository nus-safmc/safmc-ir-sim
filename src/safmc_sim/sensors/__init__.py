"""Sensing.

``base.py`` is the contract every sensor implements; ``scene.py`` is the world as a sensor
sees it; ``raycast.py`` is the geometry engine underneath. ``tof_ring.py`` and
``marker_cam.py`` are the two sensors the flown airframe carries.

Deliberately imports nothing: ``world/`` imports ``raycast`` and the sensors import
``world``, and a package ``__init__`` that pulled the sensors in would turn that into a cycle.
"""
